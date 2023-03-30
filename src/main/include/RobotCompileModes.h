#pragma once

//#define COMPETITION_MODE // only competition related dashboard output
//#define TESTING_MODE // no console output, but has dashboard output
#define DEBUG_MODE // enables console output and dashboard output


//just to guard against multiple things being enabled at the same time
#ifdef COMPETITION_MODE
    #ifdef TESTING_MODE
        #error Cannot have TESTING_MODE enabled at the same time as COMPETITION_MODE
    #endif
    #ifdef DEBUG_MODE
        #error Cannot have DEBUG_MODE enabled at the same time as COMPETITION_MODE
    #endif
#endif

// will have checked COMPETITION_MODE & TESTING_MODE
// as well as COMPETITION_MODE & DEBUG_MODE
// only need TESTING_MODE & DEBUG_MODE
#ifdef TESTING_MODE
    #ifdef DEBUG_MODE
        #error Cannot have DEBUG_MODE enabled at the same time as TESTING_MODE
    #endif
#endif

//check to make sure that ONE mode IS defines (so things will work)
#ifndef COMPETITION_MODE
    #ifndef TESTING_MODE
        #ifndef DEBUG_MODE
            #error Must have COMPETITION_MODE or TESTING_MODE or DEBUG_MODE defined
        #endif
    #endif
#endif