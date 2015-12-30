#ifndef PRIORITY_QUEUE
#define PRIORITY_QUEUE

#include <map>
#include <list>
#include <algorithm>

namespace POICS{
	template <class Key, class Value, class Comparator = std::less<Key>>
	class PriorityQueue{
		private:
			std::map<Key, std::list<Value>, Comparator> queuemap;
			int count;
		public:
			PriorityQueue(): count(0){}
			~PriorityQueue(){}

			void push(Key priority, Value obj){
				auto itr = queuemap.find(priority);
				if (itr == queuemap.end()){
					itr = queuemap.insert(make_pair(priority, std::list<Value>())).first;
				}

				itr->second.push_back(obj);
				++count;
			}

			void pop(){
				auto itr = queuemap.begin();
				itr->second.pop_front();

				if (itr->second.empty()){
					queuemap.erase(itr);
				}
				--count;
			}

			Value& front(){
				auto itr = queuemap.begin();
				return itr->second.front();
			}

			bool empty(){
				return count == 0;
			}

			bool update(Key oldPriority, Key newPriority, Value obj){
				auto itr = queuemap.find(oldPriority);
				if (itr == queuemap.end()) return false;

				auto pobj = std::find(itr->second.begin(), itr->second.end(), obj);
				if (pobj == itr->second.end()) return false;

				itr->second.erase(pobj);

				if (itr->second.empty()){
					queuemap.erase(itr);
				}

				--count;
				push(newPriority, obj);
				return true;
			}
	};
}

#endif