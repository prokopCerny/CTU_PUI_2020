//
// Created by prokopcerny on 17.04.2020.
//

#include <map>
#include <cstdint>
#include <unordered_map>
#include <set>
#include <vector>

template<class KEY,
        class COST,
        class = typename std::enable_if<std::is_arithmetic<COST>::value, bool>::type,
        typename Comparator = std::less<COST>>
class Heap {
private:
    std::vector<COST> heap;
    std::unordered_map<KEY, size_t> key_to_heap;
    std::unordered_map<size_t, KEY> heap_to_key;
    Comparator cmp;

    size_t get_parent_idx(const size_t idx) const {
        return (idx-1)/2;
    }

    size_t get_left_child_idx(const size_t idx) const {
        return (idx*2)+1;
    }

    bool less(const size_t lhs_idx, const size_t rhs_idx) {
        return cmp(heap[lhs_idx], heap[rhs_idx]);
    }

    void swap_heap(const size_t lhs_idx, const size_t rhs_idx) {
        std::swap(heap[lhs_idx], heap[rhs_idx]);
//        std::swap(key_to_heap[heap_to_key[lhs_idx]], key_to_heap[heap_to_key[rhs_idx]]);
        //alternative to the above
        key_to_heap[heap_to_key[lhs_idx]] = rhs_idx;
        key_to_heap[heap_to_key[rhs_idx]] = lhs_idx;
        std::swap(heap_to_key[lhs_idx], heap_to_key[rhs_idx]);
    }

    void sift_up(size_t current_idx) {
        size_t parent_idx = get_parent_idx(current_idx);
        while(current_idx >= 1 and
              (not less(parent_idx, current_idx))) {
            swap_heap(parent_idx, current_idx);
            current_idx = parent_idx;
            parent_idx = get_parent_idx(current_idx);
        }
    }

    void heapify(size_t current_idx) {
        size_t left_child_idx = get_left_child_idx(current_idx);
        const size_t heap_size = heap.size();
        while (left_child_idx < heap_size) {
            size_t right_child_idx = left_child_idx + 1;
            size_t best_idx = current_idx;
            if (less(left_child_idx, current_idx)) {
                best_idx = left_child_idx;
            }

            if (right_child_idx < heap_size and less(right_child_idx, best_idx)) {
                best_idx = right_child_idx;
            }

            if (best_idx != current_idx) {
                swap_heap(current_idx, best_idx);
                current_idx = best_idx;
                left_child_idx = get_left_child_idx(current_idx);
            } else {
                break;
            }
        }
    }
public:
    Heap() {}

    bool empty() const {
        return heap.empty();
    }

    bool push(KEY key, COST cost) {
        if (key_to_heap.find(key) != key_to_heap.end()) {
#ifdef MY_DEBUG
            std::cerr << "Pushing existing key!!!" << std::endl;
            exit(-1);
#endif
            return false;
        }
        size_t current_idx = heap.size();
        heap.push_back(cost);
        key_to_heap[key] = current_idx;
        heap_to_key[current_idx] = key;

        sift_up(current_idx);
        return true;
    }
    KEY pop() {
        KEY ret = heap_to_key[0];
        swap_heap(0, heap.size()-1);
        key_to_heap.erase(key_to_heap.find(ret));
        heap_to_key.erase(heap_to_key.find(heap.size()-1));
        heap.pop_back();
        if (not this->empty()) {
            heapify(0);
        }
        return ret;
    }

    bool decrease_key(KEY key, COST new_cost) {
        if (key_to_heap.find(key) == key_to_heap.end()) {
#ifdef MY_DEBUG
            std::cerr << "Decreasing non existing key!!!" << std::endl;
            exit(-1);
#endif
            return false;
        }
        heap[key_to_heap[key]] = new_cost;
        sift_up(key_to_heap[key]);
        return true;
    }
};