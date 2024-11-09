#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define TABLE_SIZE 1024
#define SAMPLE_RATE 20000

typedef struct {
    int8_t* samples;
    size_t size;
    float phase;
    float phase_inc;
} wavetable_t;

wavetable_t* create_wavetable(size_t size) {
    printf("Creating wavetable with size %zu\n", size);  // Debug print
    
    wavetable_t* wt = (wavetable_t*)malloc(sizeof(wavetable_t));
    if (!wt) {
        printf("Failed to allocate wavetable struct\n");
        return NULL;
    }

    printf("Allocating samples array\n");  // Debug print
    wt->samples = (int8_t*)malloc(size * sizeof(int8_t));
    if (!wt->samples) {
        printf("Failed to allocate samples array\n");
        free(wt);
        return NULL;
    }

    wt->size = size;
    wt->phase = 0.0f;
    wt->phase_inc = 0.0f;

    printf("Wavetable created successfully\n");  // Debug print
    return wt;
}

int load_raw_samples(const char* filename, wavetable_t* wt) {
    printf("Opening file: %s\n", filename);  // Debug print
    
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Could not open file %s\n", filename);
        return -1;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    printf("File size: %ld bytes\n", file_size);  // Debug print
    printf("Wavetable size: %zu samples\n", wt->size);  // Debug print

    // Read only up to wavetable size or file size, whichever is smaller
    size_t samples_to_read = (file_size < wt->size) ? file_size : wt->size;
    printf("Reading %zu samples\n", samples_to_read);  // Debug print

    size_t samples_read = fread(wt->samples, sizeof(int8_t), samples_to_read, file);
    printf("Actually read %zu samples\n", samples_read);  // Debug print

    // If we didn't fill the whole table, pad with zeros
    if (samples_read < wt->size) {
        printf("Padding remaining %zu samples with zeros\n", wt->size - samples_read);
        memset(wt->samples + samples_read, 0, wt->size - samples_read);
    }

    fclose(file);
    return 0;
}

// Main test program
int main() {
    printf("Starting program\n");

    // Create wavetable
    wavetable_t* wt = create_wavetable(TABLE_SIZE);
    if (!wt) {
        printf("Failed to create wavetable\n");
        return 1;
    }

    // Load samples
    if (load_raw_samples("myfile.raw", wt) != 0) {
        printf("Failed to load samples\n");
        if (wt->samples) free(wt->samples);
        free(wt);
        return 1;
    }

    // Print first few samples to verify data
    printf("First 10 samples: ");
    for (int i = 0; i < 10; i++) {
        printf("%d ", wt->samples[i]);
    }
    printf("\n");

    // Clean up
    if (wt->samples) free(wt->samples);
    free(wt);
    printf("Program completed successfully\n");

    return 0;
}