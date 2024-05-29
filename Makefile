.PHONY: clean

CXX=g++-11
accel_notify_context_switch: main.cpp
	$(CXX) -o accel_notify_context_switch main.cpp -lboost_coroutine

clean:
	rm -f accel_notify_context_switch