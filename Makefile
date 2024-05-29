.PHONY: clean

CXX=g++-11
CXX_FLAGS=-I/home/n869p538/spr-accel-profiling/interrupt_qat_dc/DML/install/include
LDD_FLAGS=-lboost_coroutine -ldml -L/home/n869p538/spr-accel-profiling/interrupt_qat_dc/DML/install/lib/
accel_notify_context_switch: main.cpp
	$(CXX) -o accel_notify_context_switch main.cpp $(LDD_FLAGS)  $(CXX_FLAGS)

clean:
	rm -f accel_notify_context_switch