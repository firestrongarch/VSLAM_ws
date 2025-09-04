module;
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>
export module lockfree;

export namespace boost {
namespace lockfree {
    using boost::lockfree::queue;
    using boost::lockfree::spsc_queue;
}

}