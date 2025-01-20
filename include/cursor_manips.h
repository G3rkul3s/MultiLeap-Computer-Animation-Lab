#ifndef CURSOR_MANIPS_H
#define CURSOR_MANIPS_H

#pragma once

#include "pch.h"

// Get the current cursor position
COORD getCursorPosition();

// Hide the console cursor
void hideCursor();

// Show the console cursor
void showCursor();

// Move the console cursor to a specific position
void moveCursor(short x, short y);

#endif // CURSOR_MANIPS_H
