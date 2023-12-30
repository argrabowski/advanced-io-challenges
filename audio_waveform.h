/*
 * audio_waveform.h
 *
 *  Created on: Apr 15, 2020
 *      Author: Gene Bogdanov
 *
 *  ECE 3849 Lab 3 Audio Waveform header
 */

#ifndef AUDIO_WAVEFORM_H_
#define AUDIO_WAVEFORM_H_

#define AUDIO_SAMPLING_RATE 16000   // [samples/sec] waveform sampling rate
extern const uint8_t gWaveform[];   // waveform array, stored in the flash
extern const size_t gWaveformSize;  // number of elements in the waveform array

#endif /* AUDIO_WAVEFORM_H_ */
