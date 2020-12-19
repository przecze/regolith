#include<chrono>
#include<iostream>

namespace simprof {

class Node {
	using clock = std::chrono::high_resolution_clock;
	public:
	Node(Node* parent, const char* name):
		parent(parent),
 		name(name) {
		enter();
	}

	Node* getChild(const char* child_name) {
		Node* current_child = child;
		while (current_child) {
			if (current_child->name == child_name) {
				current_child->enter();
				return current_child;
			}
			current_child = current_child->sibling;
		}
		auto new_child = new Node(this, child_name);
		new_child->sibling = child;
		child = new_child;
		return new_child;
	}

	Node* getParent() {
		exit();
		return parent;
	}

	void enter() {
		++calls;
		start_time = clock::now();
	}

	void exit() {
		total_time += clock::now() - start_time;
	}

	void dump_recursive(unsigned level = 0) {
		std::cout << std::string(level, '-') << name << ": ";
		std::cout << std::chrono::duration_cast<std::chrono::microseconds>(total_time).count()/1000.;
		std::cout << " ms (" << calls << " calls)" << std::endl;
		reset();
		if (child) {
			child->dump_recursive(level+1);
		}
		if (sibling) {
			sibling->dump_recursive(level);
		}
	}

  const char* name;

	protected:
	Node* parent;
	Node* child = nullptr;
	Node* sibling = nullptr;
	
	private:

	void reset() {
		calls = 0;
		total_time = clock::duration(0);
	}

	unsigned calls = 0;
	clock::time_point start_time;
	clock::duration total_time;
};


class Manager {
	static Node* current_node;
  public:
	static void start(const char* name) {
		//std::cout << "start: "<< name <<std::endl;
		current_node = current_node->getChild(name);
	}
	static void stop() {
		//std::cout << "stop: "<< current_node->name <<std::endl;
		current_node = current_node->getParent();
	}

	static void dump() {
		current_node->dump_recursive();
  }
};

class Zone
{
public:
  Zone(const char* name)
  {
    Manager::start(name);
  }
  ~Zone()
  {
    Manager::stop();
  }
};


}
