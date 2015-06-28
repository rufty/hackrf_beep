/****************************************************************************************************************************************************/
/*  Purpose:    Simple demo of using libhackrf to transmit a two-tone beep.                                                                         */
/*  Author:     Copyright (c) 2015, W.B.Hill ( M1BKF) <bugs@wbh.org> All rights reserved.                                                           */
/*  License:    Derived from hackrf_transfer, any parts written by my released into the public domain.                                              */
/****************************************************************************************************************************************************/

/*
 * This transmits at the transmit frequency, tf, +800KHz (the HamRadio APRS frequency here.)
 * At a sample rate of 8M samples/s, for 800KHz there are 10 samples per carrier wave.
 * The Mark and Space frequcies are 1200Hz and 2200Hz respectively, so 6666 and 3636 samples per wave.
 * Assuming that the signal switches between mark and space at arbitraty times, the carrier
 * will be at one of 10 possible phase angles, the carrier offset, co. The modulating signals
 * similarly at offsets of mo and so. To prevent "chirps" in the signal at the frequence of
 * transition from mark to space, there should be no discontinuity in either the modulating or
 * the carrier waves. So the mark and space waveforms are pre-calculated for all possible phase
 * angles (for speed), and changover starts from the corresponding entry, for smooth transition.
 */

// gcc -std=c11 -o hackrf_beep hackrf_beep.c -lhackrf


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <libhackrf/hackrf.h>


// Transmit frequency.
const uint64_t tf = 144000000L ;
// Sample rate.
const uint32_t sr = 8000000 ;
// Transmitter IF gain.
const unsigned int gain = 47 ;
// Depth of modulation.
const double dm = 4.0 / 3.0 ;
// Precalc, its used a bit.
const double tau = 2.0 * M_PI ;


// Lookup tables, mark and space, one wave long at 8M samples/s
int8_t mi[6666][10] ;
int8_t mq[6666][10] ;
int8_t si[3636][10] ;
int8_t sq[3636][10] ;

// Playback phase offsets for carrier, mark and space.
int co, mo, so ;

// Sample number.
long sn ;

// Mark or space?
bool ms ;

// Handle for the HackRF
static hackrf_device* device = NULL ;

// Time to give up?
volatile bool do_exit = false ;


// Dump more data to the HackRF.
int
tx_callback ( hackrf_transfer* transfer )
  {
  // How much to do?
  size_t count = transfer->valid_length ;
  // Copy it over.
  int i = 0 ;
  // Mark or space?
  if ( ms )
    {
    while ( i < count )
      {
      ( transfer->buffer ) [i++] = mi[mo][co] ;
      ( transfer->buffer ) [i++] = mq[mo][co] ;
      co++ ; co %= 10 ;
      mo++ ; mo %= 6666 ;
      }
    so =  3636 * mo / 6666 ;
    }
  else
    {
    while ( i < count )
      {
      ( transfer->buffer ) [i++] = si[so][co] ;
      ( transfer->buffer ) [i++] = sq[so][co] ;
      co++ ; co %= 10 ;
      so++ ; so %= 3636 ;
      }
    mo =  6666 * so / 3636 ;
    }
  sn += count ;
  // Every second, change.
  if ( sn >= sr )
    {
    ms = !ms ;
    sn -= sr ;
    }
  return 0 ;
  }


// Deal with interruptions.
void
sigint_callback_handler ( int signum )
  {
  fprintf ( stderr, "Caught signal %d\n", signum ) ;
  do_exit = true ;
  }


int
main ( int argc, char** argv )
  {
  /*
   * Setup.
   */
  // How did it do?
  int result ;
  // Signal and carrier angle.
  double sa, ca ;
  // Sample offsets.
  co = 0 ; mo = 0 ; so = 0 ;
  // Sample number.
  sn = 0L ;
  // Mark or space?
  ms = false ;
  // Catch signals that we want to handle gracefully.
  signal ( SIGINT, &sigint_callback_handler ) ;
  signal ( SIGILL, &sigint_callback_handler ) ;
  signal ( SIGFPE, &sigint_callback_handler ) ;
  signal ( SIGSEGV, &sigint_callback_handler ) ;
  signal ( SIGTERM, &sigint_callback_handler ) ;
  signal ( SIGABRT, &sigint_callback_handler ) ;
  // This takes a bit.
  fprintf ( stderr, "Precalculating lookup tables...\n" ) ;
  /*
   * Precalc waveforms.
   */
  // Lookup for 1200Hz.
  for ( int s = 0 ; s < 6666; s++ )
    {
    sa = s * tau / 6666.0 ;
    for ( int c = 0; c < 10; c++ )
      {
      ca = c * tau / 10.0 ;
      mi[s][c] = ( int8_t ) ( 127.0 * sin ( ca - dm * cos ( sa ) ) ) ;
      mq[s][c] = ( int8_t ) ( 127.0 * cos ( ca - dm * cos ( sa ) ) ) ;
      }
    }
  // Lookup for 2200Hz.
  for ( int s = 0 ; s < 3636; s++ )
    {
    sa = s * tau / 3636.0 ;
    for ( int c = 0; c < 10; c++ )
      {
      ca = c * tau / 10.0 ;
      si[s][c] = ( int8_t ) ( 127.0 * sin ( ca - dm * cos ( sa ) ) ) ;
      sq[s][c] = ( int8_t ) ( 127.0 * cos ( ca - dm * cos ( sa ) ) ) ;
      }
    }
  /*
   * Setup the HackRF for transmitting at full power, 8M samples/s, 144MHz
   */
  // Ok.
  fprintf ( stderr, "Setting up the HackRF...\n" ) ;
  // Initialize the HackRF.
  result = hackrf_init() ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_init() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  // Open the HackRF.
  result = hackrf_open ( &device ) ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_open() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  // Set the sample rate.
  result = hackrf_set_sample_rate_manual ( device, sr, 1 ) ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_sample_rate_set() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  // Set the filter bandwith to default.
  result = hackrf_set_baseband_filter_bandwidth ( device, hackrf_compute_baseband_filter_bw_round_down_lt ( sr ) ) ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_baseband_filter_bandwidth_set() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  // Set the gain.
  result = hackrf_set_txvga_gain ( device, gain ) ;
  result |= hackrf_start_tx ( device, tx_callback, NULL ) ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_start_tx() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  // Set the transmit frequency.
  result = hackrf_set_freq ( device, tf ) ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_set_freq() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  // Turn on the amp.
  result = hackrf_set_amp_enable ( device, ( uint8_t ) 1 ) ;
  if ( result != HACKRF_SUCCESS )
    {
    fprintf ( stderr, "hackrf_set_amp_enable() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    return EXIT_FAILURE ;
    }
  /*
   * Transmitting.
   */
  // Ready?
  fprintf ( stderr, "Transmitting, stop with Ctrl-C\n" ) ;
  // Spin until done or killed.
  while ( ( hackrf_is_streaming ( device ) == HACKRF_TRUE ) && ( do_exit == false ) ) sleep ( 1 ) ;
  /*
   * Clean up and shut down.
   */
  // What happened?
  result = hackrf_is_streaming ( device ) ;
  if ( do_exit )
    {
    printf ( "\nUser cancel, exiting...\n" ) ;
    }
  else
    {
    fprintf ( stderr, "\nExiting... hackrf_is_streaming() result: %s (%d)\n", hackrf_error_name ( result ), result ) ;
    }
  // Shut down the HackRF.
  if ( device != NULL )
    {
    result = hackrf_stop_tx ( device ) ;
    if ( result != HACKRF_SUCCESS )
      {
      fprintf ( stderr, "hackrf_stop_tx() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
      }
    result = hackrf_close ( device ) ;
    if ( result != HACKRF_SUCCESS )
      {
      fprintf ( stderr, "hackrf_close() failed: %s (%d)\n", hackrf_error_name ( result ), result ) ;
      }
    hackrf_exit() ;
    }
  // That's all, folks!!!
  return EXIT_SUCCESS ;
  }
