#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "navcontext.h"

#include <Recast.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

navcontext::navcontext() :
	message_count(0),
	text_pool_size(0)
{
	memset(messages, 0, sizeof(char*) * MAX_MESSAGES);

	resetTimers();
}

// Virtual functions for custom implementations.
void navcontext::doResetLog()
{
	message_count = 0;
	text_pool_size = 0;
}

void navcontext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (message_count >= MAX_MESSAGES)
		return;
	char* dst = &text_pool[text_pool_size];
	int n = TEXT_POOL_SIZE - text_pool_size;
	if (n < 2)
		return;
	char* cat = dst;
	char* text = dst + 1;
	const int maxtext = n - 1;
	// Store category
	*cat = (char)category;
	// Store message
	const int count = rcMin(len + 1, maxtext);
	memcpy(text, msg, count);
	text[count - 1] = '\0';
	text_pool_size += 1 + count;
	messages[message_count++] = dst;
}

void navcontext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
		accumulated_time[i] = -1;
}

void navcontext::doStartTimer(const rcTimerLabel label)
{
	start_time[label] = getPerfTime();
}

void navcontext::doStopTimer(const rcTimerLabel label)
{
	const TimeVal end_time = getPerfTime();
	const TimeVal delta_time = end_time - start_time[label];
	if (accumulated_time[label] == -1)
		accumulated_time[label] = delta_time;
	else
		accumulated_time[label] += delta_time;
}

int navcontext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return getPerfTimeUsec(accumulated_time[label]);
}

void navcontext::dump_log(const char* format, ...)
{
	// Print header.
	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");

	// Print messages
	const int TAB_STOPS[4] = { 28, 36, 44, 52 };
	for (int i = 0; i < message_count; ++i)
	{
		const char* msg = messages[i] + 1;
		int n = 0;
		while (*msg)
		{
			if (*msg == '\t')
			{
				int count = 1;
				for (int j = 0; j < 4; ++j)
				{
					if (n < TAB_STOPS[j])
					{
						count = TAB_STOPS[j] - n;
						break;
					}
				}
				while (--count)
				{
					putchar(' ');
					n++;
				}
			}
			else
			{
				putchar(*msg);
				n++;
			}
			msg++;
		}
		putchar('\n');
	}
}

int navcontext::get_log_count() const
{
	return message_count;
}

const char* navcontext::get_log_text(const int i) const
{
	return messages[i] + 1;
}
