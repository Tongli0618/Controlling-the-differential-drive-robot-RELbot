#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

#define NSEC_PER_SEC 1000000000

#define INTERVAL_NS 1000000     // Timer interval
#define CYCLES 5000             // Amount of timer iterations
#define LOAD 10000              // Amount of computational load 

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
    sprintf(filename, "Ass2_Posix_%d.txt", LOAD);

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
    // Set up a real-time signal
    sigset_t set;
    sigemptyset(&set);                      // Initialize signal set
    sigaddset(&set, SIGRTMIN);              // Add SIGRTMIN to signal set
    pthread_sigmask(SIG_BLOCK, &set, NULL); // Do not automatically handle the signal so we can wait for it

    // Define behavior when timer ends
    struct sigevent sev;
    timer_t timerid;

    sev.sigev_notify = SIGEV_SIGNAL;        // Send signal when timer expires
    sev.sigev_signo = SIGRTMIN;             // Specify which signal is sent
    sev.sigev_value.sival_ptr = &timerid;   // To know which timer caused the signal.
    
    // Create the timer
    int ret = timer_create(CLOCK_MONOTONIC, &sev, &timerid);
    
    // Check whether timer was created succesfully
    if (ret == -1) {
        perror("timer_create");
        return NULL;
    }

    // Set the timer
    struct itimerspec its;                          // Timer specification
    its.it_value.tv_sec = 0;                        // Time after which the timer first triggers
    its.it_value.tv_nsec = INTERVAL_NS;
    its.it_interval.tv_sec = 0;                     // Timer interval
    its.it_interval.tv_nsec = INTERVAL_NS;
    ret = timer_settime(timerid, 0, &its, NULL);

    // Check whether timer was set succesfully
    if (ret == -1) {
        perror("timer_settime");
        return NULL;
    }

    long long timeArrayBefore[CYCLES];  // Array containing the time before computational load of each cycle
    long long timeArrayAfter[CYCLES];   // Array containing the time after computational load of each cycle
    struct timespec before, after;      // Time before/after computational load
    int sig;                            // Signal received by sigwait

    for (int i = 0; i < CYCLES; i++) {
        // Wait for timer signal
        sigwait(&set, &sig);

        // Get current time before calculation
        clock_gettime(CLOCK_MONOTONIC, &before);
        timeArrayBefore[i] = (before.tv_sec) * 1000000000 + (before.tv_nsec);

        // Computational load
        computationalLoad();

        // Get current time after calculation
        clock_gettime(CLOCK_MONOTONIC, &after);
        timeArrayAfter[i] = (after.tv_sec) * 1000000000 + (after.tv_nsec);
    }

    // Write result to file
    writeFile(timeArrayBefore, timeArrayAfter);

    // Delete timer
    timer_delete(timerid);
    return NULL;
}

int main() {
    // Avoid the signal for the timer being handled by the main thread
    sigset_t set; 
    sigemptyset(&set);
    sigaddset(&set, SIGRTMIN);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    // Start the periodic thread
    pthread_t threadId;
    if (pthread_create(&threadId, NULL, periodicThread, NULL) != 0) {
        perror("pthread_create failed");
        return EXIT_FAILURE;
    }

    // Wait for the periodic thread to finish
    pthread_join(threadId, NULL);
    return EXIT_SUCCESS;
}
