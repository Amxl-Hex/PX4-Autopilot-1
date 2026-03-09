px4_add_module(
	MODULE modules__remaining_range
	MAIN remaining_range
	COMPILE_FLAGS
		#-DDEBUG_BUILD
	SRCS
		range_calculation.cpp
		range_calculation.hpp
	DEPENDS
		px4_work_queue
	)
