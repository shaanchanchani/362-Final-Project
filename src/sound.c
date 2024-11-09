#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// Wavetable size - must be power of 2
#define TABLE_SIZE 1024
#define SAMPLE_RATE 44100

// WAV header structure
typedef struct {
    char riff_header[4];    // Contains "RIFF"
    int32_t wav_size;       // Size of WAV
    char wave_header[4];    // Contains "WAVE"
    char fmt_header[4];     // Contains "fmt "
    int32_t fmt_chunk_size;
    int16_t audio_format;
    int16_t num_channels;
    int32_t sample_rate;
    int32_t byte_rate;
    int16_t sample_alignment;
    int16_t bit_depth;
    char data_header[4];    // Contains "data"
    int32_t data_bytes;     // Number of bytes in data
} wav_header_t;

// Wavetable structure
typedef struct {
    int16_t* samples;
    size_t size;
    float phase;
    float phase_inc;
} wavetable_t;

// Function prototypes
wavetable_t* create_wavetable(size_t size);
void destroy_wavetable(wavetable_t* wt);
int load_wav_file(const char* filename, wavetable_t* wt);
void normalize_wavetable(wavetable_t* wt);
float get_next_sample(wavetable_t* wt);
void set_frequency(wavetable_t* wt, float frequency);

// Create a new wavetable
wavetable_t* create_wavetable(size_t size) {
    wavetable_t* wt = (wavetable_t*)malloc(sizeof(wavetable_t));
    if (!wt) return NULL;

    wt->samples = (int16_t*)malloc(size * sizeof(int16_t));
    if (!wt->samples) {
        free(wt);
        return NULL;
    }

    wt->size = size;
    wt->phase = 0.0f;
    wt->phase_inc = 0.0f;

    return wt;
}

// Free wavetable memory
void destroy_wavetable(wavetable_t* wt) {
    if (wt) {
        free(wt->samples);
        free(wt);
    }
}

// Load WAV file into wavetable
int load_wav_file(const char* filename, wavetable_t* wt) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Could not open file %s\n", filename);
        return -1;
    }

    wav_header_t header;
    if (fread(&header, sizeof(header), 1, file) != 1) {
        printf("Error: Could not read WAV header\n");
        fclose(file);
        return -1;
    }

    // Verify WAV format
    if (strncmp(header.riff_header, "RIFF", 4) != 0 ||
        strncmp(header.wave_header, "WAVE", 4) != 0) {
        printf("Error: Invalid WAV format\n");
        fclose(file);
        return -1;
    }

    // Read samples
    size_t samples_to_read = wt->size;
    if (fread(wt->samples, sizeof(int16_t), samples_to_read, file) != samples_to_read) {
        printf("Warning: Could not read requested number of samples\n");
    }

    fclose(file);
    return 0;
}

// Normalize wavetable to use full 16-bit range
void normalize_wavetable(wavetable_t* wt) {
    // Find maximum absolute value
    int16_t max_val = 0;
    for (size_t i = 0; i < wt->size; i++) {
        int16_t abs_val = abs(wt->samples[i]);
        if (abs_val > max_val) max_val = abs_val;
    }

    // Scale samples
    if (max_val > 0) {
        float scale = 32767.0f / max_val;
        for (size_t i = 0; i < wt->size; i++) {
            wt->samples[i] = (int16_t)(wt->samples[i] * scale);
        }
    }
}

// Get next sample using linear interpolation
float get_next_sample(wavetable_t* wt) {
    // Calculate table indices for interpolation
    float index = wt->phase * wt->size;
    size_t index_1 = (size_t)index;
    size_t index_2 = (index_1 + 1) % wt->size;
    
    // Calculate fractional part for interpolation
    float frac = index - (float)index_1;
    
    // Linear interpolation
    float sample = wt->samples[index_1] + 
                  frac * (wt->samples[index_2] - wt->samples[index_1]);
    
    // Update phase
    wt->phase += wt->phase_inc;
    if (wt->phase >= 1.0f) wt->phase -= 1.0f;
    
    return sample / 32768.0f;  // Normalize to [-1.0, 1.0]
}

// Set frequency of playback
void set_frequency(wavetable_t* wt, float frequency) {
    wt->phase_inc = frequency * wt->size / SAMPLE_RATE;
}

// Example usage
int main() {
    // Create wavetable
    wavetable_t* wt = create_wavetable(TABLE_SIZE);
    if (!wt) {
        printf("Failed to create wavetable\n");
        return 1;
    }

    // Load WAV file
    if (load_wav_file("sample.wav", wt) != 0) {
        destroy_wavetable(wt);
        return 1;
    }

    // Normalize the samples
    normalize_wavetable(wt);

    // Set playback frequency (e.g., 440 Hz for A4)
    set_frequency(wt, 440.0f);

    // Example: Generate 1 second of audio at 440 Hz
    for (int i = 0; i < SAMPLE_RATE; i++) {
        float sample = get_next_sample(wt);
        // Here you would send the sample to your audio output
        // For example: output_buffer[i] = sample;
    }

    // Clean up
    destroy_wavetable(wt);
    return 0;
}