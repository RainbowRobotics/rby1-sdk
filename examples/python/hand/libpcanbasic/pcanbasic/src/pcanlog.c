/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcanlog.c
 * @brief Function to log stuff
 * $Id: pcanlog.c 20935 2025-01-10 09:28:55Z Fabrice $
 *
 * Copyright (C) 2001-2025  PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * PCAN is a registered Trademark of PEAK-System Germany GmbH
 *
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */
#include "pcanlog.h"

/*
 * INCLUDES
 */
#include <stdio.h>
#include <stdarg.h>		/* va_start */
#include <time.h>		/* clock_gettime */
#include <string.h>		/* strnlen */
#include <stdlib.h>		/* atexit */
#include <syslog.h>

#define MAX_PATH 260
#define strncpy_s(o,i,l) strncpy(o,i,l)
#define SYSLOG_LABEL "libpcanbasic"

/**
 * @def MIN(x,y)
 * @brief A macro that returns the minimum of @a x and @a y.
 */
#define MIN(x,y) ((x) < (y) ? (x) : (y))


/* PRIVATE FUNCTIONS DEFINITIONS	*/

/**
 * @fn const char* pcanlog_lvl_to_string(const PCANLOG_LEVEL lvl)
 * @brief Returns the PCANLOG_LEVEL value as a string.
 */
static const char* pcanlog_lvl_to_string(const PCANLOG_LEVEL lvl);

/**
 * @fn char *pcanbasic_ltrim(char *s)
 * @brief Removes leading whitespaces.
 */
static int pcanlog_lvl_to_syslog(const PCANLOG_LEVEL lvl);

#define pcanlog_vsyslog(lvl, fmt, va)	vsyslog(pcanlog_lvl_to_syslog(lvl), fmt, va)

/**
 * @fn FILE* pcanlog_get_file(void)
 * @brief Returns the instance of the opened log file (if any).
 */
static FILE* pcanlog_get_file(void);

/*
 * PRIVATE VARIABLES
 */

/* Stores the configuration of the logger */
static struct {
	int initialized;
	PCANLOG_LEVEL lvl;	/**< log level */
	FILE * f;			/**< log file descriptor (NULL for stdout) */
	int btimestamp;		/**< add a prefixed timestamp */
	char filename[MAX_PATH];
 } g_pcanlog = {0, LVL_ERROR, NULL, 1};
 

/*
 * LOCAL FUNCTIONS
 */

static const char* pcanlog_lvl_to_string(const PCANLOG_LEVEL lvl) {
	switch (lvl) {
	case LVL_VERBOSE:
		return "VERBOSE";	
	case LVL_DEBUG:
		return "DEBUG";
	case LVL_INFO:
		return "INFO";
	case LVL_NOTICE:
		return "NOTICE";
	case LVL_WARNING:
		return "WARNING";
	case LVL_ERROR:
		return "ERROR";
	case LVL_FATAL:
		return "FATAL";
	default:
		return "N/A";
	}
}

static int pcanlog_lvl_to_syslog(const PCANLOG_LEVEL lvl) {
	switch (lvl) {
	case LVL_VERBOSE:
	case LVL_DEBUG:
		return LOG_DEBUG;
	case LVL_INFO:
		return LOG_INFO;
	case LVL_NOTICE:
		return LOG_NOTICE;
	case LVL_WARNING:
		return LOG_WARNING;
	case LVL_ERROR:
		return LOG_ERR;
	case LVL_FATAL:
		return LOG_CRIT;	
	default:
		return LOG_INFO;
	}
}

int pcanlog_should_write(const PCANLOG_LEVEL lvl) {
	return (g_pcanlog.initialized > LOGTYPE_UNDEFINED && lvl <= g_pcanlog.lvl);
}

FILE* pcanlog_get_file(void) {
	FILE* f = NULL;
	if ((g_pcanlog.initialized & LOGTYPE_FILE) == LOGTYPE_FILE) {
		if (g_pcanlog.f == NULL) {
			if (g_pcanlog.filename[0] != 0) {
				g_pcanlog.f = fopen(g_pcanlog.filename, "w");
			}
			else {
				g_pcanlog.f = stderr;
			}		 
		}
		f = g_pcanlog.f;
	}
	return f;
}
 /*
  * GLOBAL FUNCTIONS
  */
void pcanlog_set(PCANLOG_TYPE enabled, const PCANLOG_LEVEL lvl, const char *filename, const int showtime) {
	if (g_pcanlog.f != NULL) {
		fflush(g_pcanlog.f);
		if (g_pcanlog.f != stderr && g_pcanlog.f != stdout)
			fclose(g_pcanlog.f);
		g_pcanlog.f = NULL;
		memset(g_pcanlog.filename, 0, sizeof(g_pcanlog.filename));
	}
	if ((g_pcanlog.initialized & LOGTYPE_SYSLOG) == LOGTYPE_SYSLOG) {
		closelog();
	}


	if (enabled == 0) {
		g_pcanlog.initialized = LOGTYPE_NONE;
		g_pcanlog.lvl = LVL_ERROR;
	}
	else {
		g_pcanlog.initialized = enabled;
		g_pcanlog.lvl = lvl;
		g_pcanlog.btimestamp = showtime;
		if ((g_pcanlog.initialized & LOGTYPE_SYSLOG) == LOGTYPE_SYSLOG) {
			int logopt = LOG_PID | LOG_CONS;
			int facility = LOG_USER;
			openlog(SYSLOG_LABEL, logopt, facility);
			setlogmask(LOG_UPTO(pcanlog_lvl_to_syslog(g_pcanlog.lvl)));
		}
		if (filename != NULL) {
			size_t len = strnlen(filename, sizeof(g_pcanlog.filename) - 1);
			strncpy_s(g_pcanlog.filename, filename, len);
			g_pcanlog.filename[len] = 0;
		}		
	}
}

void pcanlog_log(const PCANLOG_LEVEL lvl, const char *fmt, ...) {
	if (pcanlog_should_write(lvl)) {
		FILE *pfout;
		va_list ap;
		/* write to syslog */
		if ((g_pcanlog.initialized & LOGTYPE_SYSLOG) == LOGTYPE_SYSLOG) {
			va_start(ap, fmt);
			pcanlog_vsyslog(lvl, fmt, ap);
			va_end(ap);
		}
		/* write to file */
		pfout = pcanlog_get_file();
		if (pfout != NULL) {
			if (g_pcanlog.btimestamp) {
				struct timespec t;
				clock_gettime(CLOCK_MONOTONIC, &t);
				fprintf(pfout, "%010u.%06u: ", (unsigned int)t.tv_sec, (unsigned int)(t.tv_nsec / 1000));
			}
			fprintf(pfout, "%-10s ", pcanlog_lvl_to_string(lvl));
			va_start(ap, fmt);
			vfprintf(pfout, fmt, ap);
			va_end(ap);
			if (lvl == LVL_DEBUG)
				fflush(pfout);
		}
	}
}

void pcanlog_write(const PCANLOG_LEVEL lvl, const char *fmt, ...) {
	if (pcanlog_should_write(lvl)) {
		FILE *pfout;
		va_list ap;
		/* write to syslog */
		if ((g_pcanlog.initialized & LOGTYPE_SYSLOG) == LOGTYPE_SYSLOG) {
			va_start(ap, fmt);
			pcanlog_vsyslog(lvl, fmt, ap);
			va_end(ap);
		}
		/* write to file */
		pfout = pcanlog_get_file();
		if (pfout != NULL) {
			va_start(ap, fmt);
			vfprintf(pfout, fmt, ap);
			va_end(ap);
			if (lvl == LVL_DEBUG)
				fflush(pfout);
		}
	}
}
