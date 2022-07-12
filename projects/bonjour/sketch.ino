//  Illustrates how to register a PNTP Server.
//

// Headers
#include "sketch_only_statics.hpp"

#include "sketch.hpp"

#include "powernet.h"


PowernetNamespace pntp;


// Runs Once
void setup() {
    
    pntp_begin("stm32 bonjour");
};


// Runs Forever
void loop() {

	pntp_listen();
};