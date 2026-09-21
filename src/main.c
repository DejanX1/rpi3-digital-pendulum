#define _GNU_SOURCE

/*
 * Application bootstrap for the digital pendulum.
 *
 * Owns the process-wide real-time setup (locked memory, malloc tuning),
 * creates the four threads with the scheduling policies and priorities
 * from the assignment's task table, and shuts everything down cleanly on
 * SIGINT/SIGTERM so no thread is killed mid-period and the LED matrix is
 * left cleared.
 *
 * Signals are blocked here before any thread exists, so every worker
 * inherits the blocked mask and only main ever reacts to them and
 * it keeps the RT loops from being interrupted mid-period.
 */

#include "task1_imu.h"
#include "task2_joystick.h"
#include "task3_physics.h"
#include "task4_display.h"

#include <errno.h>
#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* Per-thread stack, sized deliberately rather than left at the default
 * 8MB: with mlockall(MCL_FUTURE) every stack is locked into RAM as it is
 * created, so four default stacks would pin 32MB for no reason. */
#define THREAD_STACK_EXTRA (256 * 1024)

/* Grown and touched once at startup so the RT threads never fault on a
 * fresh malloc arena later (Vjezba 9). */
#define HEAP_PREALLOC_SIZE (4 * 1024 * 1024)

struct task_spec {
    const char *name;
    void *(*entry)(void *);
    int policy;
    int priority;
};

/* The assignment's scheduling table, in one place. */
static const struct task_spec TASKS[] = {
    { "Task 1 (IMU)",      task1_imu_thread,      SCHED_FIFO,  99 },
    { "Task 2 (joystick)", task2_joystick_thread, SCHED_FIFO,  90 },
    { "Task 3 (physics)",  task3_physics_thread,  SCHED_RR,    50 },
    { "Task 4 (display)",  task4_display_thread,  SCHED_OTHER,  0 },
};

#define TASK_COUNT (sizeof(TASKS) / sizeof(TASKS[0]))

static const char *policy_name(int policy)
{
    switch (policy) {
    case SCHED_FIFO:  return "SCHED_FIFO";
    case SCHED_RR:    return "SCHED_RR";
    case SCHED_OTHER: return "SCHED_OTHER";
    default:          return "unknown";
    }
}

static void configure_rt_memory(void)
{
    long page_size;
    char *buffer;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("main: mlockall failed (continuing without locked memory)");

    mallopt(M_TRIM_THRESHOLD, -1); /* never hand heap back to the OS */
    mallopt(M_MMAP_MAX, 0);        /* keep malloc off mmap */

    page_size = sysconf(_SC_PAGESIZE);
    buffer = malloc(HEAP_PREALLOC_SIZE);
    if (buffer == NULL) {
        fprintf(stderr, "main: heap pre-allocation failed, continuing\n");
        return;
    }

    for (long offset = 0; offset < HEAP_PREALLOC_SIZE; offset += page_size)
        buffer[offset] = 0;

    free(buffer);
}

static int start_task(pthread_t *thread, const struct task_spec *spec,
                      shared_state_t *state)
{
    pthread_attr_t attr;
    struct sched_param param = { .sched_priority = spec->priority };
    int rc;

    rc = pthread_attr_init(&attr);
    if (rc != 0)
        return rc;

    if ((rc = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + THREAD_STACK_EXTRA)) != 0 ||
        (rc = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0 ||
        (rc = pthread_attr_setschedpolicy(&attr, spec->policy)) != 0 ||
        (rc = pthread_attr_setschedparam(&attr, &param)) != 0) {
        pthread_attr_destroy(&attr);
        return rc;
    }

    rc = pthread_create(thread, &attr, spec->entry, state);
    pthread_attr_destroy(&attr);
    return rc;
}

int main(void)
{
    shared_state_t state;
    pthread_t threads[TASK_COUNT];
    sigset_t signals;
    size_t started = 0;
    int sig;
    int rc;

    rc = shared_state_init(&state);
    if (rc != 0) {
        fprintf(stderr, "main: shared_state_init failed: %s\n", strerror(-rc));
        return EXIT_FAILURE;
    }

    configure_rt_memory();

    sigemptyset(&signals);
    sigaddset(&signals, SIGINT);
    sigaddset(&signals, SIGTERM);
    rc = pthread_sigmask(SIG_BLOCK, &signals, NULL);
    if (rc != 0) {
        fprintf(stderr, "main: pthread_sigmask failed: %s\n", strerror(rc));
        shared_state_destroy(&state);
        return EXIT_FAILURE;
    }

    for (size_t i = 0; i < TASK_COUNT; i++) {
        rc = start_task(&threads[i], &TASKS[i], &state);
        if (rc != 0) {
            fprintf(stderr, "main: could not start %s: %s\n",
                    TASKS[i].name, strerror(rc));
            if (rc == EPERM)
                fprintf(stderr, "main: real-time scheduling needs privileges"
                                " - run this with sudo.\n");
            break;
        }

        printf("started %-18s %-12s priority %d\n",
               TASKS[i].name, policy_name(TASKS[i].policy), TASKS[i].priority);
        started++;
    }

    if (started == TASK_COUNT) {
        printf("\nDigital pendulum running. Press Ctrl+C to stop.\n");
        sigwait(&signals, &sig);
        printf("\nsignal %d received, shutting down...\n", sig);
    }

    /* Also reached when a thread failed to start: the ones already running
     * are stopped and joined before we exit, none are left behind. */
    shared_state_request_stop(&state);
    for (size_t i = 0; i < started; i++)
        pthread_join(threads[i], NULL);

    shared_state_destroy(&state);
    printf("all threads joined, exiting cleanly.\n");

    return (started == TASK_COUNT) ? EXIT_SUCCESS : EXIT_FAILURE;
}
