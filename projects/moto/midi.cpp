#include "odrive_main.h"

#ifdef __MIDI_HPP

extern "C" {
  #include "miditones.h"
}

// This holds the playable bytestream that the midi parser creates
// It should always be of smaller size than the midi file but I do not know 
// what the ratio is yet.
volatile uint8_t midi_player_bytestream_[25000] = { 0 };

extern Midi *midi_;

Midi::Midi(Config_t& config) : config_(config) {

    // Zero-fill the modulation table so we don't have garbage in it
    memset(midi_modulation_table_, 0, sizeof(midi_modulation_table_));

    // Parse midi file stored in flash into a playable bytestream
    parse_midi((void *)&midi_file_, sizeof(midi_file_), (void *)&midi_player_bytestream_, sizeof(midi_player_bytestream_));

    // Calculate a LUT for MIDI frequencies
    for(int i = 0; i < 128; i++) {
        note_periods_[i] = (uint16_t)((float)midi_player_loop_frequency_ / (float)note_frequencies_float_[i]);
    }
}

//-----------------------------------------------
// Start playing a note on a particular channel
//-----------------------------------------------
void Midi::tune_playnote (uint8_t channel, uint8_t note) {

    // If the channel has something in it already then return
    if(channel_scheduler_[channel] != 0) {
    	return;
    }

    // Reject invalid notes
	if(note > 128) {
		return;
	}
	
    // Save the note period in the scheduler 
    channel_scheduler_[channel] = note_periods_[note];

    // Write all bits of the channel in the modulation table to 1
    // Notice how we use 2 * channel to do some time-dithering of the notes
    // This leads to some ambiguity at the start of the modulation table
    // but when you start reading the table at [midi_modulation_table_underscan_] 
    // it should be fine
    for(uint16_t j = 2 * channel; j < midi_modulation_table_size_; j+= channel_scheduler_[channel]) {

        // Bits = Channels
        midi_modulation_table_[j] = midi_modulation_table_[j] | 1UL << channel;

        // Amplify bass
        if(channel_scheduler_[channel] > midi_bass_boost_) {
            midi_modulation_table_[j+1] = midi_modulation_table_[j+1] | 1UL << channel;
        }
    }
}

//-----------------------------------------------
// Stop playing a note on a particular channel
//-----------------------------------------------
void Midi::tune_stopnote (uint8_t channel) {

    // If the channel scheduler has nothing in it then return
    if(channel_scheduler_[channel] == 0) {
    	return;
    }

    // Otherwise set all bits of the channel in the period saved in the scheduler back to 0
    for(uint16_t j = 2 * channel; j < midi_modulation_table_size_; j+= channel_scheduler_[channel]) {
        midi_modulation_table_[j] = midi_modulation_table_[j] & ~(1UL << channel);
        
        // Amplify bass
        if(channel_scheduler_[channel] > midi_bass_boost_) {
            midi_modulation_table_[j+1] = midi_modulation_table_[j+1] & ~(1UL << channel);
        }
    }

    channel_scheduler_[channel] = 0;
}

//-----------------------------------------------
// Start playing a score
//-----------------------------------------------
void Midi::tune_playscore (const char *score) {
    
	// Stop song if already playing
	if (tune_playing_) {
		tune_stopscore();
	}

	played_first_note_ 	= false;
    song_start_ 		= 0;
    volume_present_ 	= 0;

    song_cursor_ = song_start_;

    tune_stepscore();

 	// release the kraken 
    tune_playing_ = true; 
}

void Midi::tune_stepscore (void) {
    uint8_t cmd;
    uint8_t opcode;
    uint8_t channel;
    uint8_t note;
    uint16_t duration;

    //  Do score commands until a "wait" is found, or the score is stopped.
    //  This is called initially from tune_playcore, but then is called
    //  from the interrupt routine when waits expire.

    #define CMD_PLAYNOTE    0x90	// play a note: low nibble is generator #, note is next byte 
    #define CMD_STOPNOTE    0x80	// stop a note: low nibble is generator # 
    #define CMD_INSTRUMENT  0xc0    // change instrument; low nibble is generator #, instrument is next byte 
    #define CMD_RESTART	    0xe0	// restart the score from the beginning 
    #define CMD_STOP	    0xf0	// stop playing 

    // if CMD < 0x80, then the other 7 bits and the next byte are a 15-bit big-endian number of msec to wait 

    while (true) {
        song_cursor_++;
        cmd = midi_player_bytestream_[song_cursor_];

        // wait count in msec. 
        if (cmd < 0x80) { 
            song_cursor_++;
            duration = ((unsigned)cmd << 8) | midi_player_bytestream_[song_cursor_];
            
			if(played_first_note_) {
            	wait_toggle_count_ =  duration;
			}

            if (wait_toggle_count_ == 0) {
				wait_toggle_count_ = 1;
			}

            break;
        }

        opcode = cmd & 0xf0;
        channel   = cmd & 0x0f;
        
		switch(opcode) {
			case CMD_STOPNOTE:
				tune_stopnote (channel);
				break;

			case CMD_PLAYNOTE:
				// Increment song cursor
				song_cursor_++;
				note = midi_player_bytestream_[song_cursor_]; 

				// ignore volume byte if present (we're only doing 1 bit modulation)
				if (volume_present_) {
					song_cursor_++; 
				}

				tune_playnote (channel, note);
				played_first_note_ = true;
				break;

			case CMD_INSTRUMENT:
				// ignore this, we only have one instrument (the motor!)
            	song_cursor_++; 
				break;
       
	   		case CMD_RESTART:
				played_first_note_ = false;
            	song_cursor_ = song_start_;
				break;
		
			case CMD_STOP: 
            	tune_playing_ = false;
            	break;

			default:
				break;
    	}

		// Leave loop if the song is over!
		if(tune_playing_ == false) {
			break;
		}
    }
}

//-----------------------------------------------
// Stop playing a score
//-----------------------------------------------
void Midi::tune_stopscore (void) {
    uint8_t channel_bits;

    for (channel_bits = 0; channel_bits < sizeof(midi_modulation_table_[0]); channel_bits++) {
        tune_stopnote(channel_bits);
    }

    tune_playing_ = false;
	played_first_note_ = false;
}

/*static void tune_timer_wrapper(TimerHandle_t myTimer)
{
    void * class_pointer = pvTimerGetTimerID(myTimer);

    // Call function of the passed class instance
    reinterpret_cast<Midi*>(class_pointer)->tune_timer();
}
*/

// We keep this running always and use it to time score waits, whether or not it is playing a note.
void Midi::tune_timer() {

	--wait_toggle_count_;

    // execute commands if wait toggles have reached zero
    if (tune_playing_ && wait_toggle_count_ <= 0) {
        tune_stepscore ();
    }
}

static void midi_thread_wrapper(void * class_pointer) {

    reinterpret_cast<Midi*>(class_pointer)->tune_playscore( 0 );

    while(true) {
        reinterpret_cast<Midi*>(class_pointer)->tune_timer();
        osDelay(1);
    }
}

void Midi::start_thread() {
    osThreadDef(midi_thread, midi_thread_wrapper, osPriorityNormal, 0, 512);

    thread_id_ = osThreadCreate(osThread(midi_thread), this);
}

#endif