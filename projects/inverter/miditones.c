#define _GNU_SOURCE

#include <stdio.h>
#include <stdbool.h>
#include <limits.h>

#include <miditones.h>

struct midi_header {
   int8_t MThd[4];
   uint32_t header_size;
   uint16_t format_type;
   uint16_t number_of_tracks;
   uint16_t time_division; 
};

struct track_header {
   int8_t MTrk[4];
   uint32_t track_size; 
};


// max tone generators: tones we can play simultaneously
#define MAX_TONEGENS       8        

// default number of tone generators
#define DEFAULT_TONEGENS   8          

// max number of MIDI tracks we will process
#define MAX_TRACKS         24         

// the track MIDI uses for percussion sounds
#define PERCUSSION_TRACK   9           

// MIDI-specified number of channels
#define NUM_CHANNELS       16          

// max number of notes playing simultaneously on a channel
#define MAX_CHANNELNOTES   16          

// the MIDI-specified default tempo_ in usec/beat 
#define DEFAULT_TEMPO      500000L     

// the MIDI-specified default ticks per beat 
#define DEFAULT_BEATTIME   240         

FILE *outfile_;

uint8_t *buffer_, *hdrptr_;
uint16_t buflen_;
uint8_t num_tracks_;
uint8_t tracks_done_ = 0;
uint8_t num_tonegens_ = DEFAULT_TONEGENS;
uint8_t num_tonegens_used_ = 0;
uint8_t instrument_changes_ = 0;
uint16_t note_on_commands_ = 0;
uint16_t notes_skipped_ = 0;
uint16_t events_delayed_ = 0;
uint16_t stopnotes_without_playnotes_ = 0;
uint16_t playnotes_without_stopnotes_ = 0;
uint16_t sustainphases_skipped_ = 0;
uint16_t sustainphases_done_ = 0;
uint16_t consecutive_delays_ = 0;
uint16_t noteinfo_overflow_ = 0;
uint16_t noteinfo_notfound_ = 0;

bool last_output_was_delay_ = false;

static const bool instrumentoutput_ = false;
static const bool percussion_ignore_ = true;
static const bool percussion_translate_ = false;

static const bool generate_restart_ = true;
static const bool strategy1_ = false;
static const bool strategy2_ = false;

static const bool volume_output_ = false;


// bit mask of channels to process
unsigned channel_mask_ = 0xffff;

// optional chromatic note shift for output file
int keyshift_ = 0;

// events this close get merged together to save bytestream space
unsigned long delaymin_usec_ = 10;

// release time in usec for silence at the end of notes
unsigned long releasetime_usec_ = 0;

 // minimum note time in usec after the release is deducted
unsigned long notemin_usec_ = 10;

// the high volume attack phase lasts this time, if not 0 (only for -v)
unsigned long attacktime_usec_ = 0;

// the longest note to which the attack/sustain profile is used (only for -v)
unsigned long attacknotemax_usec_ = ULONG_MAX;

// the percent of attack volume for the sustain phase (only for -v)
int sustainlevel_pct_ = 50;

long int outfile_bytecount_ = 0;

unsigned int ticks_per_beat_ = DEFAULT_BEATTIME;

// the current processing time in ticks
volatile unsigned long timenow_ticks_ = 0;

// the current processing time in usec
long timenow_usec_ = 0;

// when, in ticks, we last updated timenow_usec_ using the current tempo_
unsigned long timenow_usec_updated_ = 0;

// the time we last output, in usec
long output_usec_ = 0;

// the leftover usec < 1000 still to be used for a "delay"
unsigned int output_deficit_usec_ = 0;

// current global tempo_ in usec/beat
unsigned long tempo_ = DEFAULT_TEMPO;

// how many times we changed the global tempo_
int tempo_changes_ = 0;

// how many delays were saved because of non-zero merge time
long int delays_saved_ = 0;

// Void for stashing unused variables
int the_void_ = 0;

// output bytestream commands, which are also stored in track_status.cmd
#define CMD_PLAYNOTE    0x90    // play a note: low nibble is generator #, note is next uint8_t 
#define CMD_STOPNOTE    0x80    // stop a note: low nibble is generator #
#define CMD_INSTRUMENT  0xc0    // change instrument; low nibble is generator #, instrument is next uint8_t
#define CMD_RESTART     0xe0    // restart the score from the beginning
#define CMD_STOP        0xf0    // stop playing
#define CMD_TEMPO       0xFE    // tempo_ in usec per quarter note ("beat")
#define CMD_TRACKDONE   0xFF    // no more data left in this track




/****************  utility routines  **********************/

void assert(bool condition, char *msg) {
   //if (!condition) {
   //   fprintf(stderr, "*** internal assertion error: %s\n", msg);
   //   if (logfile) fprintf(logfile, "*** internal assertion error: %s\n", msg);
   //   exit(8); } 
   
   if(!condition) {
      while(1) {
         __asm__ ("BKPT");   
      }
   }
}

// fatal MIDI file format error
void midi_error(char *msg, uint8_t *bufptr) {
   //fprintf(stderr, "---> MIDI file error at position %04X (%d): %s\n",
   //        (uint16_t)(bufptr - buffer_), (uint16_t)(bufptr - buffer_), msg);
   //byte *ptr = bufptr - 16;   // print some bytes surrounding the error
   //if (ptr < buffer_) ptr = buffer_;
   //for (; ptr <= bufptr + 16 && ptr < buffer_ + buflen_; ++ptr)
   //   fprintf(stderr, ptr == bufptr ? " [%02X]  " : "%02X ", *ptr);
   //fprintf(stderr, "\n");
   //exit(8); 
   while(1) {
      __asm__ ("BKPT");   
   }
}

// portable string length
int strlength (const char *str) {
   int i;
   for (i = 0; str[i] != '\0'; ++i);
   return i; 
}

// match a constant character sequence 
int charcmp (const char *buf, const char *match) {
   int len, i;

   len = strlength (match);

   for (i = 0; i < len; ++i) {
      if (buf[i] != match[i]) return 0;
   }
   
   return 1; 
}

// check that we have a specified number of bytes left in the buffer_ 
void chk_bufdata (uint8_t *ptr, unsigned long int len) {
   if ((unsigned) (ptr + len - buffer_) > buflen_) {
      midi_error ("data missing", ptr);
   }
}

// fetch big-endian numbers
uint16_t rev_short (uint16_t val) {
   return ((val & 0xff) << 8) | ((val >> 8) & 0xff);
}

uint32_t rev_long (uint32_t val) {
   return (((rev_short ((uint16_t) val) & 0xffff) << 16) | (rev_short ((uint16_t) (val >> 16)) & 0xffff)); 
}


//******* structures for recording track, channel, and tone generator status

// Note that the tempo_ can change as notes are played, maybe many times.
// In order to keep track of how long notes are to play, we have to incrementally
// accumulate the duration of all playing notes every time the tempo_ changes, and
// then one final time when the "stop note" event occurs.

struct noteinfo {                   // everything we might care about as a note plays
   uint32_t time_usec;             // when it starts or stops, in absolute usec since song start
   int track, channel, note, instrument, volume; // all the nitty-gritty about it
};

struct tonegen_status {         // current status of a tone generator
   bool playing;                // is it playing?
   bool stopnote_pending;       // are we due to issue a stop note command?
   struct noteinfo note;        // if so, the details of the note being played
} tonegen[MAX_TONEGENS] = { 0 };

struct track_status {           // current status of a MIDI track
   uint8_t *trkptr;             // ptr to the next event we care about
   uint8_t *trkend;             // ptr just past the end of the track
   unsigned long time;          // what time we're at in the score, in ticks
   unsigned long tempo_;         // the last tempo_ set on this track
   int preferred_tonegen;       // for strategy2_: try to use this generator
   uint8_t cmd;                    // next CMD_xxxx event coming up
   uint8_t chan, note, volume;     // if it is CMD_PLAYNOTE or CMD_STOPNOTE, the note info
   uint8_t last_event;             // the last event, for MIDI's "running status"
} track[MAX_TRACKS] = { 0 };

struct channel_status {          // current status of a channel
   int instrument;               // which instrument this channel currently plays
   bool note_playing[MAX_CHANNELNOTES]; // slots for notes that are playing on this channel
   struct noteinfo notes_playing[MAX_CHANNELNOTES]; // information about them
} channel[NUM_CHANNELS] = { 0 };


#define QUEUE_SIZE 100    // maximum number of note play/stop commands we queue

// the format of each queue entry
struct queue_entry 
{      
   uint8_t cmd;              // CMD_PLAY or CMD_STOP
   struct noteinfo note;  // info about the note, including the action time
} queue[QUEUE_SIZE];

uint8_t queue_numitems   = 0;
uint8_t queue_oldest_ndx = 0;
uint8_t queue_newest_ndx = 0;


/************** output reorder queue routines ******************

We queue commands to be issued at arbitrary times and sort them in time order. We flush
them out, allocating tone generators and creating the output bytestream, only as needed
to get more space in the queue. As we do that, we generate the "delay" commands required.

All the queuing must be done with a microsecond time base, not ticks, because the tempo_,
which controls the duration of ticks, may (and does!) change while notes are being played.

We currently define "uint32_ts" by typedef as uint32_t, which is enough for songs as long
as 71 minutes. If longer songs are required, change it to uint64_t.

We assume that "unsigned long" variables are big enough to track the total number of ticks
in a song. If they are 32 bits, the typical 480 ticks/beat and 500,000 usec/tick imply
960 ticks/second, for a song length of 51 days. Even if the song plays much faster, it's
still plenty long.
*/


// find an idle tone generator we can use, returns -1 if there isn't one
int find_idle_tgen(struct noteinfo *np) {

   struct tonegen_status *tg;
   
   int tgnum;
   
   bool foundgen = false;
   
   // first, is this note already playing on this channel?
   for (tgnum = 0; tgnum < num_tonegens_; ++tgnum) {

      tg = &tonegen[tgnum];
      
      if (tg->playing && tg->note.note == np->note && tg->note.channel == np->channel) {

         // this must be the start of the sustain phase of a playing note
         ++playnotes_without_stopnotes_;
         
         // Playnote without stopnote, tgen [tgnum], note [np]
         foundgen = true;
         break; 
      } 
   }

   // try to use the same tone generator that this track used last time
   if (!foundgen && strategy2_) {

      struct track_status *trk = &track[np->track];

      tg = &tonegen[trk->preferred_tonegen];

      if (!tg->playing) {
         tgnum = trk->preferred_tonegen;
         foundgen = true; 
      } 
   }

   // if not, then try for a free tone generator that had been playing the same instrument we need
   if (!foundgen) {    
      
      for (tgnum = 0; tgnum < num_tonegens_; ++tgnum) {
         
         tg = &tonegen[tgnum];

         if (!tg->playing && tg->note.instrument == np->instrument) {
            foundgen = true;
            break; 
         } 
      }
   }

   // if not, then try for any free tone generator
   if (!foundgen) {
      for (tgnum = 0; tgnum < num_tonegens_; ++tgnum) {
         tg = &tonegen[tgnum];
         if (!tg->playing) {
            foundgen = true;
            break; 
         } 
      }
   }

   if (foundgen) {
      return tgnum;
   }

   return -1; 
}

// remove the oldest queue entry
void remove_queue_entry(int ndx) {

   struct queue_entry *q = &queue[ndx];

   if (q->cmd == CMD_STOPNOTE) 
   {
      // find the tone generator playing this note, and record a pending stop
      int tgnum;

      for (tgnum = 0; tgnum < num_tonegens_; ++tgnum) {

         struct tonegen_status *tg = &tonegen[tgnum];
         
         if (tg->playing && tg->note.note == q->note.note && tg->note.channel == q->note.channel) {

            // Stop note needed unless another start note follows
            tg->stopnote_pending = true; 

            // free the tg to be reallocated, but note the stop time in case
            tg->playing = false; 

            // the tg doesn't get used and we generate it
            tg->note.time_usec = q->note.time_usec;  

            // Pending stop tgen [tgnum] for [&q->note]
            break; 
         } 
      }
      if (tgnum >= num_tonegens_) {
         // If we exited the loop without finding the generator playing this note, presumably it never started
         // because there weren't any free tone generators. Is there some assertion we can use to verify that?

         // Stopnote without playnote, [&q->note] 
         ++stopnotes_without_playnotes_;
      } 
   }

   // CMD_PLAYNOTE
   else { 
      assert(q->cmd == CMD_PLAYNOTE, "bad cmd in remove_queue_entry");

      int tgnum = (int) find_idle_tgen(&q->note);

      struct tonegen_status *tg = &tonegen[tgnum];

      // we found a tone generator we can use
      if (tgnum >= 0) {    
         
         if (tgnum + 1 > num_tonegens_used_) {
            num_tonegens_used_ = tgnum + 1;
         }
         
         // it's a new instrument for this generator
         if (tg->note.instrument != q->note.instrument) { 
            
            // Tone gen [tgnum] changed to instrument [tg->note.instrument]
            tg->note.instrument = q->note.instrument;
            ++instrument_changes_;
            
            // output a "change instrument" command
            if (instrumentoutput_) { 
               putc(CMD_INSTRUMENT | tgnum, outfile_);
               putc(tg->note.instrument, outfile_);
               outfile_bytecount_ += 2; 
            } 
         }

         // Play tgen [tgnum], note is [&q->note]
         tg->playing = true;

         // don't bother to issue "stop note"
         tg->stopnote_pending = false;

         // structure copy of note info
         tg->note = q->note;  
         
         track[tg->note.track].preferred_tonegen = tgnum;

         ++note_on_commands_;
         
         last_output_was_delay_ = false;

         putc(CMD_PLAYNOTE | tgnum, outfile_);
         putc(tg->note.note, outfile_);
         outfile_bytecount_ += 2;

         if (volume_output_) {
            putc(tg->note.volume, outfile_);
            outfile_bytecount_ +=1; 
         }
      }

      else {
         // No free generator; skipping [&q->note], at [output_usec_ / 1000, output_usec_ % 1000]
         ++notes_skipped_; 
      } 
   } 
}

// output a delay command
void generate_delay(unsigned long delta_msec) { 
   
   if (delta_msec > 0) {

      assert(delta_msec <= 0x7fff, "time delta too big");

      // This is a consecutive delay, of [delta_msec] milliseconds
      if (last_output_was_delay_) {
         ++consecutive_delays_;
      }
      
      last_output_was_delay_ = true;
      
      // output a 15-bit delay in big-endian format
      putc((uint8_t)(delta_msec >> 8), outfile_);
      putc((uint8_t)(delta_msec & 0xff), outfile_);
      outfile_bytecount_ += 2; 
   } 
}

// output all queue elements which are at the oldest time or at most "delaymin" later
void pull_queue(void) {

   // the oldest time
   uint32_t oldtime = queue[queue_oldest_ndx].note.time_usec;
   assert(oldtime >= output_usec_, "oldest queue entry goes backward in pull_queue");

   unsigned long delta_usec = (oldtime - output_usec_) + output_deficit_usec_;
   unsigned long delta_msec = delta_usec / 1000;

   output_deficit_usec_ = delta_usec % 1000;
   
   // if time has advanced beyond the merge threshold, output a delay
   if (delta_usec > (unsigned long)delaymin_usec_) {

      if (delta_msec > 0) {
         generate_delay(delta_msec);
      }
      output_usec_ = oldtime; 
   }
   else if (delta_msec > 0) {
      ++delays_saved_;
   }

   // output and remove all entries at the same (oldest) time in the queue
   // or which are only delaymin newer
   do {
      remove_queue_entry(queue_oldest_ndx);
      if (++queue_oldest_ndx >= QUEUE_SIZE) {
         queue_oldest_ndx = 0;
      }
      --queue_numitems; 
   }
   while (queue_numitems > 0 && queue[queue_oldest_ndx].note.time_usec <= oldtime + (uint32_t)delaymin_usec_);

   // do any "stop notes" still needed to be generated?
   for (int tgnum = 0; tgnum < num_tonegens_; ++tgnum) {

      struct tonegen_status *tg = &tonegen[tgnum];

      // got one
      if (tg->stopnote_pending) { 
         last_output_was_delay_ = false;

         putc(CMD_STOPNOTE | tgnum, outfile_);
         outfile_bytecount_ += 1; 

         // Stop tgen [tgnum], note [&tg->note]
         tg->stopnote_pending = false;
         tg->playing = false; 
      } 
   } 
}

// empty the queue
void flush_queue(void) { 
   while (queue_numitems > 0) {
      pull_queue(); 
   }
}

// queue a "note on" or "note off" command
void queue_cmd(uint8_t cmd, struct noteinfo *np) {
   
   if (queue_numitems == QUEUE_SIZE) {
      pull_queue();
   }

   assert(queue_numitems < QUEUE_SIZE, "no room in queue");

   uint32_t horizon = output_usec_ + output_deficit_usec_;

   // don't allow revisionist history
   if (np->time_usec < horizon) { 
      //  Event delayed by [horizon - np->time_usec] usec because queue is too small\n",
      np->time_usec = horizon;
      ++events_delayed_; 
   }
   
   int ndx;

   // If queue is empty; restart it
   if (queue_numitems == 0) {
      ndx = queue_oldest_ndx = queue_newest_ndx = queue_numitems = 0; 
   }

   // Otherwise find a place to insert the new entry in time order
   // this is a stable incremental insertion sort
   else {  
      // start with newest, since we are most often newer
      ndx = queue_newest_ndx; 

      // search backwards for something as new or older
      while (queue[ndx].note.time_usec > np->time_usec) {

         // none: we are oldest; add to the start
         if (ndx == queue_oldest_ndx) { 
            
            if (--queue_oldest_ndx < 0) {
               queue_oldest_ndx = QUEUE_SIZE - 1;
            }

            ndx = queue_oldest_ndx;
            goto insert; 
         }

         if (--ndx < 0) {
            ndx = QUEUE_SIZE - 1; 
         }
      }

      // we are to insert the new item after "ndx", so shift all later entries down, if any
      int from_ndx, to_ndx;

      if (++queue_newest_ndx >= QUEUE_SIZE) {
         queue_newest_ndx = 0;
      }
      
      to_ndx = queue_newest_ndx;

      while (true) {
         if ((from_ndx = to_ndx - 1) < 0) {
            from_ndx = QUEUE_SIZE - 1;
         }
         if (from_ndx == ndx) {
            break;
         }
         
         // structure copy
         queue[to_ndx] = queue[from_ndx]; 
         to_ndx = from_ndx; 
      }

      if (++ndx >= QUEUE_SIZE) {
         ndx = 0; 
      }
   }

// store the item at ndx
insert: 
   ++queue_numitems;

   // file in the queue entry
   queue[ndx].cmd = cmd;

   // structure copy of the note
   queue[ndx].note = *np;
}


/**************  process the MIDI file header  *****************/

void process_file_header (void) {

   struct midi_header *hdr;
   unsigned int time_division;

   chk_bufdata (hdrptr_, sizeof (struct midi_header));
   hdr = (struct midi_header *) hdrptr_;

   if (!charcmp ((char *) hdr->MThd, "MThd")) {
      midi_error ("Missing 'MThd'", hdrptr_);
   }

   num_tracks_ = rev_short (hdr->number_of_tracks);
   time_division = rev_short (hdr->time_division);

   if (time_division < 0x8000) {
      ticks_per_beat_ = time_division;
   }
   else {
      ticks_per_beat_ = ((time_division >> 8) & 0x7f) /* SMTE frames/sec */ *(time_division & 0xff);     /* ticks/SMTE frame */
   }
   
   // Header size [uint32_t(hdr->header_size)]
   // Format type [uint16_t(hdr->format_type)]
   // Number of tracks [num_tracks_]
   // Time division [time_division]
   // Ticks/beat = [ticks_per_beat_]
   
   // point past header to track header, presumably.
   hdrptr_ += rev_long (hdr->header_size) + 8;   

   return; 
}

void process_track_header (int tracknum) {

   struct track_header *hdr;
   unsigned long tracklen;

   chk_bufdata (hdrptr_, sizeof (struct track_header));

   hdr = (struct track_header *) hdrptr_;

   if (!charcmp ((char *) (hdr->MTrk), "MTrk")) {
      midi_error ("Missing 'MTrk'", hdrptr_);
   }

   tracklen = rev_long (hdr->track_size);

   // point past header
   hdrptr_ += sizeof (struct track_header);

   chk_bufdata (hdrptr_, tracklen);

   track[tracknum].trkptr = hdrptr_;

   // point to the start of the next track
   hdrptr_ += tracklen;          

   // the point past the end of the track
   track[tracknum].trkend = hdrptr_;     
}

unsigned long get_varlen (uint8_t ** ptr) {  
   // get a MIDI-style integer
   // Get a 1-4 uint8_t variable-length value and adjust the pointer past it.
   // These are a succession of 7-bit values with a MSB bit of zero marking the end */
   unsigned long val = 0;

   for (int i = 0; i < 4; ++i) {
      
      uint8_t b = *(*ptr)++;
      val = (val << 7) | (b & 0x7f);
      
      if (!(b & 0x80)) {
         return val; 
      }
   }
   
   return val; 
}

/***************  Process the MIDI track data  ***************************/

// Skip in the track for the next "note on", "note off" or "set tempo_" command and return.

void find_next_note (int tracknum) {

   unsigned long int delta_ticks;
   int event, chan;
   int note;
   int velocity;
   int controller; 
   int pressure;
   int pitchbend;
   int instrument;
   int meta_cmd, meta_length;
   unsigned long int sysex_length;

   // Our track status structure
   struct track_status *t = &track[tracknum];  
   
   while (t->trkptr < t->trkend) {

      delta_ticks = get_varlen (&t->trkptr);
      t->time += delta_ticks;

      // using "running status": same event as before
      if (*t->trkptr < 0x80) {
         event = t->last_event;  
      }

      // otherwise get new "status" (event type)
      else {
         event = *t->trkptr++; 
      }

      // meta-event
      if (event == 0xff) { 
         meta_cmd = *t->trkptr++;
         meta_length = get_varlen(&t->trkptr);
         
         switch (meta_cmd) {
            case 0x00:
               // Sequence number rev_short (*(unsigned short *) t->trkptr)
               break;
            case 0x01:
               // Description
               goto show_text;
            case 0x02:
               // Copyright
               goto show_text;
            case 0x03:
               // Track name
               // Incredibly, MIDI has no standard for recording the name of the piece!
               // Track 0's "trackname" is often used for that

               goto show_text;

            case 0x04:
               // Instrument name
               goto show_text;

            case 0x05:
               // Lyric 
               goto show_text;

            case 0x06:
               // Marked point
               goto show_text;

            case 0x07:
               // Cue point 
               goto show_text;

            case 0x08:
               // Program name
               goto show_text;

            case 0x09:
               // Device (port) name
            show_text:
            /*
            if (logparse) {
               fprintf (logfile, "meta cmd %02X, length %d, %s: \"", meta_cmd, meta_length, tag);
               for (int i = 0; i < meta_length; ++i) {
                  int ch = t->trkptr[i];
                  fprintf (logfile, "%c", isprint (ch) ? ch : '?'); }
               fprintf (logfile, "\"\n"); 
            }*/
            break;
         
         case 0x20:
            // Channel prefix
            break;
         
         case 0x21:
            // MIDI port
            break;

         case 0x2f:
            // End of track
            break;

         case 0x51:
            // Tempo: 3 uint8_t big-endian integer, not a varlen integer!    
            t->cmd      = CMD_TEMPO;
            t->tempo_    = rev_long (*(uint32_t *) (t->trkptr - 1)) & 0xffffffL;
            t->trkptr   += meta_length;
            return;

         case 0x54:
            // SMPTE offset %08" PRIx32 rev_long (*(uint32_t *) t->trkptr));
            break;

         case 0x58:
            // Time signature %08" PRIx32 rev_long (*(uint32_t *) t->trkptr));
            break;

         case 0x59:
            // Key signature rev_short (*(unsigned short *) t->trkptr));
            break;

         case 0x7f:
            // Sequencer data

         default:
            // Unknown meta cmd [meta_cmd], length [meta_length], tag [tag]
            break; 
         }

         t->trkptr += meta_length; 
      }

      else if (event < 0x80) {
         midi_error ("Unknown MIDI event type", t->trkptr);
      }

      else {
         // Remember "running status" if not meta or sysex event
         if (event < 0xf0) {
            t->last_event = event;      
         }

         t->chan = chan = event & 0xf;

         switch (event >> 4) {
            case 0x8: // note off
               t->note = *t->trkptr++;
               t->volume = *t->trkptr++;
               
               note_off:   
               // Note [t->note] off, channel [chan], volume [t->volume]
               if ((1 << chan) & channel_mask_ && (!percussion_ignore_ || chan != PERCUSSION_TRACK)) { 

                  // if no insruments, force all notes to channel 0
                  if (!instrumentoutput_) {
                     t->chan = 0; 
                  }

                  // Stop processing and return
                  t->cmd = CMD_STOPNOTE;   
                  return; 
               }
               break;

            case 0x9: 
               // note on
               t->note = *t->trkptr++;
               t->volume = *t->trkptr++;

               // Some scores use note-on with zero velocity for off!
               if (t->volume == 0) {
                  goto note_off;
               }

               if ((1 << chan) & channel_mask_ && (!percussion_ignore_ || chan != PERCUSSION_TRACK)) {
                  // If no insruments, force all notes to channel 0
                  if (!instrumentoutput_) {
                     t->chan = 0; 
                  }

                  // stop processing and return
                  t->cmd = CMD_PLAYNOTE;    
                  return; 
               }
               break;

            case 0xa: 
               // key pressure
               // Channel [chan] note [note] has key pressure [velocity]
               note = *t->trkptr++;
               velocity = *t->trkptr++;
               break;

            case 0xb: 
               // Channel [chan] change control value of [controller] to [velocity]
               controller = *t->trkptr++;
               velocity = *t->trkptr++;
               break;

            case 0xc: 
               // program patch, ie which instrument
               // Channel [chan] program patch to instrument [instrument]
               instrument = *t->trkptr++;
               channel[chan].instrument = instrument;
               break;

            case 0xd: 
               // Channel [chan] after-touch pressure is [pressure]
               pressure = *t->trkptr++;
               break;

            case 0xe: 
               // Pitch wheel change to [pitchbend]
               pitchbend = *t->trkptr++;
               pitchbend |= (*t->trkptr++ << 7);
               break;

            case 0xf: 
               // SysEx [event] with [sysex_length] bytes
               sysex_length = get_varlen (&t->trkptr);
               t->trkptr += sysex_length;
               break;

            default:
               midi_error ("Unknown MIDI command", t->trkptr); 


            // If we don't use these variables for something the compiler complains
            // yes we could turn the unused-variable warning off but I'd rather stash 
            // the variables into the void so we don't get sloppy.            
            the_void_ = pressure;
            the_void_ = instrument;
            the_void_ = velocity;
            the_void_ = note;
            the_void_ = controller;
         }  
      } 
   }

   t->cmd = CMD_TRACKDONE;   //no more events to process on this track
   ++tracks_done_;
}


void process_track_data(void) {

   // Do...while there are still track notes to process...
   do {

      /* Find the track with the earliest event time, and process it's event.

      A potential improvement: If there are multiple tracks with the same time,
      first do the ones with STOPNOTE as the next command, if any.  That would
      help avoid running out of tone generators.  In practice, though, most MIDI
      files do all the STOPNOTEs first anyway, so it won't have much effect.

      Usually we start with the track after the one we did last time (tracknum),
      so that if we run out of tone generators, we have been fair to all the tracks.
      The alternate "strategy1_" says we always start with track 0, which means
      that we favor early tracks over later ones when there aren't enough tone generators. */

      struct track_status *trk;
      int count_tracks = num_tracks_;
      
      // in ticks
      volatile unsigned long earliest_time = 0x7fffffff;
      
      int tracknum = 0;
      int earliest_tracknum = 0;

      if (strategy1_) {
         tracknum = num_tracks_;       
      }
      
      // Beyond the end, so we start with track 0

      do {
         if (++tracknum >= num_tracks_) {
            tracknum = 0;
         }

         trk = &track[tracknum];
         
         if (trk->cmd != CMD_TRACKDONE && trk->time < earliest_time) {
            earliest_time = trk->time;
            earliest_tracknum = tracknum; 
         } 

      } while (--count_tracks);
      
      // The track we picked
      tracknum = earliest_tracknum;
      trk = &track[tracknum];

      assert(earliest_time >= timenow_ticks_, "time went backwards in process_track_data");
      
      // we make it the global time
      timenow_ticks_ = earliest_time; 
      
      timenow_usec_ += (uint64_t)(timenow_ticks_ - timenow_usec_updated_) * tempo_ / ticks_per_beat_;

      // usec version is updated based on the current tempo_
      timenow_usec_updated_ = timenow_ticks_; 
      
      // the channel info, if play or stop
      struct channel_status *cp = &channel[trk->chan];  

      // change the global tempo_, which affects future usec computations
      if (trk->cmd == CMD_TEMPO) {

         if (tempo_ != trk->tempo_) {
            ++tempo_changes_;
            tempo_ = trk->tempo_; 
         }
         
         // Tempo set to [tempo_] usec/quarter note
         find_next_note(tracknum); 
      }
      
      else { 
         // should be PLAYNOTE or STOPNOTE

         if (percussion_translate_ && trk->chan == PERCUSSION_TRACK) {
            // @todo maybe move percussion notes up to 128..255
            trk->note += 128;  
         }

         else {  
            // shift notes as requested
            trk->note += keyshift_;
            if (trk->note < 0) trk->note = 0;
            if (trk->note > 127) trk->note = 127; 
         }

         if (trk->cmd == CMD_STOPNOTE) {
            // find the noteinfo for this note -- which better be playing -- in the channel status
            int ndx;

            for (ndx = 0; ndx < MAX_CHANNELNOTES; ++ndx) {
               if (cp->note_playing[ndx] && cp->notes_playing[ndx].note == trk->note && cp->notes_playing[ndx].track == tracknum) {
                  break; 
               }
            }

            if (ndx >= MAX_CHANNELNOTES) {
               // presumably the array overflowed on input
               ++noteinfo_notfound_; 
            }

            else {
               // Analyze the sustain and release parameters. We might generate another "note on"
               // command with reduced volume, and/or move the stopnote command earlier than now.
               struct noteinfo *np = &cp->notes_playing[ndx];

               // it has the start time in it
               unsigned long duration_usec = timenow_usec_ - np->time_usec; 
               unsigned long truncation;
               
               if (duration_usec <= notemin_usec_) {
                  truncation = 0;
               }
               else if (duration_usec < releasetime_usec_ + notemin_usec_) {
                  truncation = duration_usec - notemin_usec_;
               }
               else {
                  truncation = releasetime_usec_;
               }
               if (attacktime_usec_ > 0 && duration_usec < attacknotemax_usec_) {
                  if (duration_usec - truncation > attacktime_usec_) { 
                     // do a sustain phase
                     if ((np->volume = np->volume * sustainlevel_pct_ / 100) <= 0) {
                        np->volume = 1;
                     }
                     
                     // adjust time to be when sustain phase starts
                     np->time_usec += attacktime_usec_; 
                     queue_cmd(CMD_PLAYNOTE, np);
                     ++sustainphases_done_; 
                  }
                  else {
                     ++sustainphases_skipped_;
                  } 
               }

               // adjust time to be when the note stops
               np->time_usec = timenow_usec_ - truncation;
               queue_cmd(CMD_STOPNOTE, np);
               cp->note_playing[ndx] = false; 
            }

            find_next_note(tracknum); 
         }

         // Process only one "start note", so other tracks get a chance at tone generators
         else if (trk->cmd == CMD_PLAYNOTE) { 
            // find an unused noteinfo slot to use
            int ndx;  
            
            for (ndx = 0; ndx < MAX_CHANNELNOTES; ++ndx) {
               if (!cp->note_playing[ndx]) break; 
            }

            // too many simultaneous notes? 
            if (ndx >= MAX_CHANNELNOTES) {
               // No noteinfo slot to queue track [tracknum] note [trk->note] on channel [trk->chan]
               ++noteinfo_overflow_; 
            }
            else {
               // assign note to us
               cp->note_playing[ndx] = true;  
               struct noteinfo *pn = &cp->notes_playing[ndx];

               // fill it in
               pn->time_usec = timenow_usec_; 
               pn->track = tracknum;
               pn->channel = trk->chan;
               pn->note = trk->note;
               pn->instrument = cp->instrument;
               pn->volume = trk->volume;
               queue_cmd(CMD_PLAYNOTE, pn); 
            }

            find_next_note(tracknum); 
         }

         // use up the note
         else assert(false, "bad cmd in process_track_data"); 
      } 
   } while (tracks_done_ < num_tracks_);

   // empty the output queue and generate the end-of-score command
   flush_queue();
   
   assert(timenow_usec_ >= output_usec_, "time deficit at end of song");
   
   generate_delay((timenow_usec_ - output_usec_) / 1000);
   
   putc(generate_restart_ ? CMD_RESTART : CMD_STOP, outfile_);
   
   outfile_bytecount_ +=1; 
}


/*********************  main  ****************************/

int parse_midi (void * input_file, long input_file_size, void * output_file, long output_file_size) {

   // Files
   buffer_ = input_file;
   buflen_ = input_file_size;

   if (!buffer_) {
      return 1;
   }

   // Create the output file      
   outfile_ = fmemopen(output_file, output_file_size, "w");

   if (!outfile_) {
      // Error: Unable to open output file
      // Unable to allocate [buflen_] bytes for the file?
      return 1; 
   }
  
   // Point to the file and track headers
   hdrptr_ = buffer_; 

   // Process the MIDI file header
   process_file_header();

   // Processing [num_tracks_] tracks
   if (num_tracks_ > MAX_TRACKS) {
      midi_error ("Too many tracks", buffer_);
   }

   // Initialize for processing of all the tracks
   tempo_ = DEFAULT_TEMPO;

   for (int tracknum = 0; tracknum < num_tracks_; ++tracknum) {
      track[tracknum].tempo_ = DEFAULT_TEMPO;
      process_track_header (tracknum);

      // position to the first note on/off
      find_next_note (tracknum);
   }

   // Do all the tracks interleaved, like a 1950's multiway merge
   process_track_data();
   
   fclose(outfile_); 

   return 0;    
}
