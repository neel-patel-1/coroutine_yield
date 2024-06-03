.PHONY: clean all

CXX=g++-11
CXX_FLAGS=-I/home/n869p538/spr-accel-profiling/interrupt_qat_dc/DML/install/include -I/home/n869p538/spr-accel-profiling/interrupt_qat_dc -I/home/n869p538/spr-accel-profiling/interrupt_qat_dc/qatlib/quickassist/include
LDD_FLAGS=-lboost_coroutine -ldml -lqat -lusdm -L/home/n869p538/spr-accel-profiling/interrupt_qat_dc/DML/install/lib/ -L/home/n869p538/spr-accel-profiling/interrupt_qat_dc/qatlib/.libs
FLAGS= -DBREAKDOWN

all: baseline-ctx-switch single-request-latency accel_notify_context_switch baseline-cpu

accel_notify_context_switch: main.cpp
	$(CXX) -o accel_notify_context_switch main.cpp $(LDD_FLAGS)  $(CXX_FLAGS) $(FLAGS)

baseline-ctx-switch: baseline-ctx-switch.cpp
	$(CXX) -o baseline-ctx-switch baseline-ctx-switch.cpp $(LDD_FLAGS)  $(CXX_FLAGS) $(FLAGS)

baseline-cpu: baseline-cpu.cpp
	$(CXX) -o baseline-cpu baseline-cpu.cpp $(LDD_FLAGS)  $(CXX_FLAGS) $(FLAGS) -lz

single-request-latency: single-request-latency.cpp
	$(CXX) -o single-request-latency single-request-latency.cpp $(LDD_FLAGS)  $(CXX_FLAGS) $(FLAGS)

clean:
	rm -f accel_notify_context_switch baseline-ctx-switch single-request-latency