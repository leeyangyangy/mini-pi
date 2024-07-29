/* vi: set sw=4 ts=4: */
/*
 * Unicode support routines.
 *
 * Copyright (C) 2010 Denys Vlasenko
 *
 * Licensed under GPLv2, see file LICENSE in this source tree.
 */
#include "libbb.h"
#include "unicode.h"

const char* FAST_FUNC printable_string2(uni_stat_t *stats, const char *str)
{
	char *dst;
	const char *s;

	s = str;
	while (1)
	{
		unsigned char c = *s;
		if (c == '\0')
		{
			/* 99+% of inputs do not need conversion */
			if (stats)
			{
				stats->byte_count = (s - str);
				stats->unicode_count = (s - str);
				stats->unicode_width = (s - str);
			}
			return str;
		}
		if (c < ' ')
			break;
		if (c >= 0x7f)
			break;
		s++;
	}
}

const char* FAST_FUNC printable_string(const char *str)
{
	return printable_string2(NULL, str);
}
