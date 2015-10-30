#include <windows.h>
#include <deque>

class Timer{
	public:
		Timer() {QueryPerformanceFrequency(&frequency);}
		void Start() {QueryPerformanceCounter(&start);}
		double End() {
			QueryPerformanceCounter(&end);
			return double(end.QuadPart - start.QuadPart) / frequency.QuadPart;
		}
	private:
		LARGE_INTEGER frequency;
		LARGE_INTEGER start;
		LARGE_INTEGER end;
};

class Profile{
	public:
		Profile(int s) : maxt(s) {t = NULL;}
		~Profile() {delete t;}
		void Start() {if (!t) t = new Timer; t->Start();}
		void End() {
			if (!t) return;
			times.emplace_front(t->End());
			delete t;
			t = NULL;
			if (times.size() > maxt) times.pop_back();
		}
		double Get() {
			double result = 0;
			for (int i = 0; i < times.size(); i++)
				result += times[i];
			return result / times.size();
		}
	private:
		Timer* t;
		int maxt;
		std::deque<double> times;
};

