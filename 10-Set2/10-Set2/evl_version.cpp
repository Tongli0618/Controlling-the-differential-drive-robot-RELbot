#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

#define NSEC_PER_SEC 1000000000

#define INTERVAL_NS 1000000     // Timer interval
#define CYCLES 5000             // Amount of timer iterations
#define LOAD 10000              // Amount of computational load 

// Function to add nanoseconds to timespec
void timespec_add_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

// Function for dummy computational load
void computationalLoad() {
    volatile double dummy = 0.0;
    
    for (int j = 0; j < LOAD; ++j) {
        dummy += j * 0.1;
    }
}

// Function to write measured timing to a file
void writeFile(long long timeArrayBefore[], long long timeArrayAfter[]) {
    char filename[64];
    sprintf(filename, "Ass2_EVL_%d.txt", LOAD);

    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        perror("fopen");
    }
    for (int i = 0; i < CYCLES-1; i++) {
        const long long period = timeArrayAfter[i+1] - timeArrayAfter[i];
        const long long load = timeArrayAfter[i] - timeArrayBefore[i];
        fprintf(file, "Before: %lld ns, After: %lld ns, Load: %lld ns, Period: %lld ns\n", timeArrayBefore[i], timeArrayAfter[i], load, period);
    }
    fclose(file);
}

void* periodicThread(void* arg) {
    // Mount thread to the EVL Core
    int ret = evl_attach_self("my RT work thread");
    
    // Check whether mount was succesful
    if (ret < 0) {
        fprintf(stderr, "Failed to attach to EVL core: %s\n", strerror(-ret));
        return NULL;
    }

    // Create the timer
    int tmfd = evl_new_timer(EVL_CLOCK_MONOTONIC);

    // Check whether timer was created succesfully
    if (tmfd < 0) {
        fprintf(stderr, "Failed to create EVL timer: %s\n", strerror(-tmfd));
        return NULL;
    }

    // Set the timer
    struct timespec now;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
    timespec_add_ns(&now, INTERVAL_NS);         // First trigger INTERVAL_NS ms after now
    struct itimerspec its;                      // Timer specification
    its.it_value = now;                         // Time at which timer is first triggered
    its.it_interval.tv_sec = 0;                 // Timer interval
    its.it_interval.tv_nsec = INTERVAL_NS;
    ret = evl_set_timer(tmfd, &its, NULL);

    // Check whether timer was set succesfully
    if (ret) {
        fprintf(stderr, "Failed to set EVL timer: %s\n", strerror(-ret));
        return NULL;
    }
    
    long long timeArrayBefore[CYCLES];      // Array containing the time before computational load of each cycle
    long long timeArrayAfter[CYCLES];       // Array containing the time after computational load of each cycle
    __u64 ticks;                            // Counter for how many time ticks have been missed
    struct timespec before, after;     // Time now/before/after computational load

    for (int i = 0; i < CYCLES; i++) {
        // Block until the interval
        ret = oob_read(tmfd, &ticks, sizeof(ticks));
        
        // Check whether blocking succeeded 
        if (ret < 0) {
            fprintf(stderr, "Failed to wait on EVL timer: %s\n", strerror(-ret));
            break;
        }

        // Read clock before
        evl_read_clock(EVL_CLOCK_MONOTONIC, &before);
        timeArrayBefore[i] = (before.tv_sec) * 1000000000 + (before.tv_nsec);

        // Computational load
        computationalLoad();
        
        // Read clock after
        evl_read_clock(EVL_CLOCK_MONOTONIC, &after);
        timeArrayAfter[i] = (after.tv_sec) * 1000000000 + (after.tv_nsec);
    }

    // Write result to file
    writeFile(timeArrayBefore, timeArrayAfter);

    // Detach from EVL core
    evl_detach_self();
    return NULL;
}

int main() {
    // Initialize attributes to set scheduling properties
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Set the scheduling policy to FIFO
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // Explicitly set the thread scheduling policy and parameters
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    // Set scheduling priority to the maximum priority for FIFO
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);

    // Start the periodic thread
    pthread_t threadId;
    if (pthread_create(&threadId, &attr, periodicThread, NULL) != 0) {
        perror("pthread_create failed");
        return EXIT_FAILURE;
    }

    pthread_join(threadId, NULL);   // Wait for the thread to finish
    pthread_attr_destroy(&attr);    // Clean up thread attributes
    return EXIT_SUCCESS;
}
