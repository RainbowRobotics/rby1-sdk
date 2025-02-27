/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcanlog.h
 * @brief Function prototypes to log stuff
 * $Id: pcanlog.h 20935 2025-01-10 09:28:55Z Fabrice $
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

#ifndef __PCANLOG_H__
#define __PCANLOG_H__

/**
 * Defines log verbosity
 */
typedef enum {
	LVL_VERBOSE = 8,	/**< verbose debug-level */
	LVL_DEBUG = 7,		/**< debug-level */
	LVL_INFO = 6,		/**< informational */
	LVL_NOTICE = 5,		/**< normal but significant condition */
	LVL_WARNING = 4,	/**< warning conditions */
	LVL_ERROR = 3,		/**< error conditions */
	LVL_FATAL = 2		/**< critical conditions */
} PCANLOG_LEVEL;

/**
 * Defines logging mechanism
 */
typedef enum {
	LOGTYPE_UNDEFINED = 0,	/**< logging unset */
	LOGTYPE_NONE = -1,		/**< disabled */
	LOGTYPE_SYSLOG = 1,		/**< syslog */
	LOGTYPE_FILE = 2,		/**< PCANBasic.log */
	LOGTYPE_ALL = LOGTYPE_SYSLOG | LOGTYPE_FILE		/**< syslog & PCANBasic.log */
} PCANLOG_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn void pcanlog_set(PCANLOG_LEVEL lvl, char *filename, int showtime)
 * @brief Configures the logging system.
 *
 * @param[in] enabled logging status
 * @param[in] lvl The maximum level to be displayed
 * @param[in] filename The filename to write the log
 * @param[in] showtime State to prefix the log with a timestamp
 */
void pcanlog_set(PCANLOG_TYPE enabled, const PCANLOG_LEVEL lvl, const char *filename, const int showtime);

/**
 * @fn void pcanlog_log(PCANLOG_LEVEL lvl, char *fmt, ...)
 * @brief Logs an entry (with a timestamp if optien is set)
 *
 * @param[in] lvl level of the log
 * @param[in] fmt Formatted string
 */
void pcanlog_log(const PCANLOG_LEVEL lvl, const char *fmt, ...);

/**
 * @fn void pcanlog_log(PCANLOG_LEVEL lvl, char *fmt, ...)
 * @brief Writes a raw message in the log
 *
 * @param[in] lvl level of the log
 * @param[in] fmt Formatted string
 */
void pcanlog_write(const PCANLOG_LEVEL lvl, const char *fmt, ...);

/**
 * @fn int pcanlog_should_write(const PCANLOG_LEVEL lvl)
 * @brief States if lvl is enough to allow a log entry.
 *
 * @param[in] lvl level of the log
 */
int pcanlog_should_write(const PCANLOG_LEVEL lvl);

#ifdef __cplusplus
};
#endif

#endif /* __PCANLOG_H__ */
