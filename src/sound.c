#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define TABLE_SIZE 1024
#define SAMPLE_RATE 20000  // Changed to match your 20kHz rate

typedef struct {
    int8_t* samples;       // Changed to 8-bit
    size_t size;
    float phase;
    float phase_inc;
} wavetable_t;

// Create a new wavetable
wavetable_t* create_wavetable(size_t size) {
    wavetable_t* wt = (wavetable_t*)malloc(sizeof(wavetable_t));
    if (!wt) return NULL;

    wt->samples = (int8_t*)malloc(size * sizeof(int8_t));  // Changed to 8-bit
    if (!wt->samples) {
        free(wt);
        return NULL;
    }

    wt->size = size;
    wt->phase = 0.0f;
    wt->phase_inc = 0.0f;

    return wt;
}

// Load raw 8-bit samples
int load_raw_samples(const char* filename, wavetable_t* wt) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Could not open file %s\n", filename);
        return -1;
    }

    // Read raw 8-bit samples directly
    size_t samples_read = fread(wt->samples, sizeof(int8_t), wt->size, file);
    if (samples_read != wt->size) {
        printf("Warning: Only read %zu of %zu samples\n", samples_read, wt->size);
    }

    fclose(file);
    return 0;
}

// Get next sample with interpolation
float get_next_sample(wavetable_t* wt) {
    float index = wt->phase * wt->size;
    size_t index_1 = (size_t)index;
    size_t index_2 = (index_1 + 1) % wt->size;
    
    float frac = index - (float)index_1;
    float sample = wt->samples[index_1] + 
                  frac * (wt->samples[index_2] - wt->samples[index_1]);
    
    wt->phase += wt->phase_inc;
    if (wt->phase >= 1.0f) wt->phase -= 1.0f;
    
    return sample / 128.0f;  // Convert to -1.0 to 1.0 range (8-bit range is -128 to 127)
}

// Set playback frequency
void set_frequency(wavetable_t* wt, float frequency) {
    wt->phase_inc = frequency * wt->size / SAMPLE_RATE;
}

// Clean up
void destroy_wavetable(wavetable_t* wt) {
    if (wt) {
        free(wt->samples);
        free(wt);
    }
}

// Example usage
int main() {
    wavetable_t* wt = create_wavetable(TABLE_SIZE);
    if (!wt) {
        printf("Failed to create wavetable\n");
        return 1;
    }

    // Load your raw audio samples
    if (load_raw_samples("myfile.raw", wt) != 0) {
        destroy_wavetable(wt);
        return 1;
    }

    // Set frequency to 440 Hz
    set_frequency(wt, 440.0f);

    // Generate some samples
    for (int i = 0; i < 1000; i++) {
        float sample = get_next_sample(wt);
        // Use sample here...
    }

    destroy_wavetable(wt);
    return 0;
}