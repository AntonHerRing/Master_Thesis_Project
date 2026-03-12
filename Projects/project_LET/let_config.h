#ifndef LET_CONFIG_H
#define LET_CONFIG_H

/**
 * @brief Flag indicates if the no-copy communication is implemented or not. 
 * The no-copy communication
 */
#define LET_NO_COPY_COMMUNICATION   1

/**
 * @brief Flag indicates if the LET task records runtime overheads.
 */
#define LET_RECORD_OVERHEAD         0

/**
 * @brief Defines for the core affinity masks on two cores.
 */
#define CORE0   0x1
#define CORE1   0x2
#define CORE1_2 0x3

#if LET_RECORD_OVERHEAD == 1
/*
 * Add a platform specific implementation for the overhead measurement.
 */
#error "No implementation for overhead measurement provided!"
#else 
/*
 * Empty defines to exclude overhead measurement if not used.
 */
#define vLetRecInit()
#define vLetRecExecStart()
#define vLetRecExecStop()
#endif

#endif /* LET_CONFIG_H */