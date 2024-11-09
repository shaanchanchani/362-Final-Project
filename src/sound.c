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
    printf("Creating wavetable with size %zu\n", size);
    
    wavetable_t* wt = (wavetable_t*)malloc(sizeof(wavetable_t));
    if (!wt) {
        printf("Failed to allocate wavetable struct\n");
        return NULL;
    }

    wt->samples = (int8_t*)malloc(size * sizeof(int8_t));
    if (!wt->samples) {
        printf("Failed to allocate samples array\n");
        free(wt);
        return NULL;
    }

    wt->size = size;
    wt->phase = 0.0f;
    wt->phase_inc = 0.0f;
    return wt;
}

// Added offset parameter to skip initial silence
int load_raw_samples(const char* filename, wavetable_t* wt, size_t offset) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Could not open file %s\n", filename);
        return -1;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    printf("File size: %ld bytes\n", file_size);

    // Skip the offset
    if (offset > 0) {
        if (offset >= file_size) {
            printf("Error: Offset larger than file size\n");
            fclose(file);
            return -1;
        }
        fseek(file, offset, SEEK_SET);
        printf("Skipped %zu bytes\n", offset);
    }

    // Read samples
    size_t remaining_bytes = file_size - offset;
    size_t samples_to_read = (remaining_bytes < wt->size) ? remaining_bytes : wt->size;
    size_t samples_read = fread(wt->samples, sizeof(int8_t), samples_to_read, file);
    
    printf("Read %zu samples\n", samples_read);

    // Print some statistics about the samples
    int8_t min_sample = 127;
    int8_t max_sample = -128;
    float average = 0;
    
    for (size_t i = 0; i < samples_read; i++) {
        if (wt->samples[i] < min_sample) min_sample = wt->samples[i];
        if (wt->samples[i] > max_sample) max_sample = wt->samples[i];
        average += wt->samples[i];
    }
    average /= samples_read;

    printf("Sample statistics:\n");
    printf("Min: %d\n", min_sample);
    printf("Max: %d\n", max_sample);
    printf("Average: %.2f\n", average);

    // If we didn't fill the whole table, pad with zeros
    if (samples_read < wt->size) {
        printf("Padding remaining %zu samples with zeros\n", wt->size - samples_read);
        memset(wt->samples + samples_read, 0, wt->size - samples_read);
    }

    fclose(file);
    return 0;
}

// Function to dump samples to a text file for verification
void dump_samples(const char* filename, wavetable_t* wt) {
    FILE* file = fopen(filename, "w");
    if (!file) return;
    
    for (size_t i = 0; i < wt->size; i++) {
        fprintf(file, "%d\n", wt->samples[i]);
    }
    fclose(file);
}

int main() {
    printf("Starting program\n");

    wavetable_t* wt = create_wavetable(TABLE_SIZE);
    if (!wt) {
        printf("Failed to create wavetable\n");
        return 1;
    }

    // Try different offsets until we find good audio data
    // Start at 1000 samples in (50ms at 20kHz)
    if (load_raw_samples("myfile.raw", wt, 1000) != 0) {
        printf("Failed to load samples\n");
        free(wt->samples);
        free(wt);
        return 1;
    }

    // Print first few samples in both decimal and hex
    printf("First 20 samples:\n");
    for (int i = 0; i < 20; i++) {
        printf("%3d (0x%02x) ", wt->samples[i], (unsigned char)wt->samples[i]);
        if ((i + 1) % 5 == 0) printf("\n");
    }

    // Save all samples to a file for inspection
    dump_samples("samples.txt", wt);
    printf("\nSaved all samples to samples.txt\n");

    if (wt->samples) free(wt->samples);
    free(wt);
    return 0;
}