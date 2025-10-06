#pragma once

#include <Recast.h>
#include <RecastDump.h>
#include <PerfTimer.h>

class navcontext : public rcContext
{
	TimeVal start_time[RC_MAX_TIMERS];
	TimeVal accumulated_time[RC_MAX_TIMERS];

	static const int MAX_MESSAGES = 1000;
	const char* messages[MAX_MESSAGES];
	int message_count;
	static const int TEXT_POOL_SIZE = 8000;
	char text_pool[TEXT_POOL_SIZE];
	int text_pool_size;

public:
	navcontext();

	/// Dumps the log to stdout.
	void dump_log(const char* format, ...);
	/// Returns number of log messages.
	int get_log_count() const;
	/// Returns log message text.
	const char* get_log_text(const int i) const;

protected:
	virtual void doResetLog();
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	virtual void doResetTimers();
	virtual void doStartTimer(const rcTimerLabel label);
	virtual void doStopTimer(const rcTimerLabel label);
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const;
};