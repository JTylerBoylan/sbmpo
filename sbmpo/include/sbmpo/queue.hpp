#ifndef SBMPO_QUEUE_HPP
#define SBMPO_QUEUE_HPP

#include <vector>
#include <functional>

namespace sbmpo {

    class Queue {

        public:

            std::vector<int> heap;
            std::function<bool(int,int)> compare;

            Queue(std::function<bool(int,int)> comp) : heap(), compare(comp) {}

            void swap(int *a, int *b) {
                int temp = *b;
                *b = *a;
                *a = temp;
            }

            void heapify(int i) {

                int size = heap.size();
                
                int min = i;
                int l = 2 * i + 1;
                int r = 2 * i + 2;
                if (l < size && compare(heap[l], heap[min]))
                    min = l;
                if (r < size && compare(heap[r], heap[min]))
                    min = r;

                if (min != i) {
                    swap(&heap[i], &heap[min]);
                    heapify(min);
                }
            }

            void insert(int idx) {
                int size = heap.size();
                if (size == 0) {
                    heap.push_back(idx);
                } else {
                    heap.push_back(idx);
                    for (int i = size / 2 - 1; i >= 0; i--) {
                        heapify(i);
                    }
                }
            }

            void remove(int idx) {
                int size = heap.size();
                int i;
                for (i = 0; i < size; i++) {
                    if (idx == heap[i])
                    break;
                }
                swap(&heap[i], &heap[size - 1]);

                heap.pop_back();
                for (int i = size / 2 - 1; i >= 0; i--) {
                    heapify(i);
                }
            }

            int pop() {
                int back = heap.back();
                heap.pop_back();
                return back;
            }

            bool empty() {
                return heap.size() == 0;
            }

    };

}


#endif