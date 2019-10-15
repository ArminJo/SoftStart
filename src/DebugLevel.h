/*
 * DebugLevel.h
 * Include to propagate debug levels
 *
 *  Created on: 27.09.2019
 *      Author: Armin
 */

#ifndef DEBUGLEVEL_H_
#define DEBUGLEVEL_H_

// Propagate debug level
#ifdef TRACE
#define DEBUG
#endif
#ifdef DEBUG
#define INFO
#endif
#ifdef INFO
#define WARN
#endif
#ifdef WARN
#define ERROR
#endif

#endif /* DEBUGLEVEL_H_ */

#pragma once
