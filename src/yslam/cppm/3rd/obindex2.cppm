export module obindex2;
import std;
import cv;
export namespace obindex2 {

// 1. Binary Descriptor
class BinaryDescriptor {
public:
    // Constructors
    explicit BinaryDescriptor(const unsigned nbits = 256);
    explicit BinaryDescriptor(const unsigned char* bits, unsigned nbytes);
    explicit BinaryDescriptor(const cv::Mat& desc);
    explicit BinaryDescriptor(const BinaryDescriptor& bd);
    // Destructor
    virtual ~BinaryDescriptor()
    {
        delete[] bits_;
    };

    // Methods
    inline void set(int nbit)
    {
        // Detecting the correct byte
        int nbyte = nbit / 8;
        int nb = 7 - (nbit % 8);

        // Setting the bit
        bits_[nbyte] |= 1 << nb;
    }

    inline void reset(int nbit)
    {
        // Detecting the correct byte
        int nbyte = nbit / 8;
        int nb = 7 - (nbit % 8);

        // Setting the bit
        bits_[nbyte] &= ~(1 << nb);
    }

    inline int size()
    {
        return static_cast<int>(size_in_bits_);
    }

    inline static double distHamming(const BinaryDescriptor& a,
        const BinaryDescriptor& b)
    {
        int hamming = cv::hal::normHamming(a.bits_, b.bits_, a.size_in_bytes_);
        return static_cast<double>(hamming);
    }

    // Operator overloading
    inline bool operator==(const BinaryDescriptor& d)
    {
        int hamming = cv::hal::normHamming(bits_, d.bits_, size_in_bytes_);
        return hamming == 0;
    }

    inline bool operator!=(const BinaryDescriptor& d)
    {
        int hamming = cv::hal::normHamming(bits_, d.bits_, size_in_bytes_);
        return hamming != 0;
    }

    inline BinaryDescriptor& operator=(const BinaryDescriptor& other)
    {
        // Clearing previous memory
        if (bits_ != nullptr) {
            delete[] bits_;
        }

        // Allocating new memory
        size_in_bits_ = other.size_in_bits_;
        size_in_bytes_ = other.size_in_bytes_;
        bits_ = new unsigned char[size_in_bytes_];
        std::memcpy(bits_, other.bits_, sizeof(unsigned char) * size_in_bytes_);

        return *this;
    }

    inline BinaryDescriptor& operator&=(const BinaryDescriptor& other)
    {
        unsigned size = other.size_in_bytes_;
        for (unsigned i = 0; i < size; i++) {
            bits_[i] = bits_[i] & other.bits_[i];
        }

        return *this;
    }

    inline BinaryDescriptor& operator|=(const BinaryDescriptor& other)
    {
        unsigned size = other.size_in_bytes_;
        for (unsigned i = 0; i < size; i++) {
            bits_[i] = bits_[i] | other.bits_[i];
        }

        return *this;
    }

    cv::Mat toCvMat()
    {
        cv::Mat m = cv::Mat::zeros(1, size_in_bytes_, cv::MAT_8U);
        unsigned char* d = m.ptr<unsigned char>(0);
        std::memcpy(d, bits_, sizeof(unsigned char) * size_in_bytes_);
        return m.clone();
    }
    // std::string toString();

    // For simplicity, we made it public, but you should use the public methods
    unsigned char* bits_;
    unsigned size_in_bytes_;
    unsigned size_in_bits_;
};

using BinaryDescriptorPtr = std::shared_ptr<BinaryDescriptor>;
using BinaryDescriptorSet = std::unordered_set<BinaryDescriptorPtr>;
using BinaryDescriptorSetPtr = std::shared_ptr<BinaryDescriptorSet>;

BinaryDescriptor::BinaryDescriptor(const unsigned nbits)
{
    if (nbits % 8 == 0) {
        throw std::invalid_argument("The number of bits must be a multiple of 8");
    }
    size_in_bits_ = nbits;
    size_in_bytes_ = static_cast<unsigned>(nbits / 8);
    bits_ = new unsigned char[size_in_bytes_];

    // Initializing the bits
    for (unsigned i = 0; i < size_in_bytes_; i++) {
        bits_[i] = 0;
    }
}

BinaryDescriptor::BinaryDescriptor(const unsigned char* bits, unsigned nbytes)
{
    if (nbytes % 8 == 0) {
        throw std::invalid_argument("The number of bytes must be a multiple of 8");
    }
    size_in_bits_ = nbytes * 8;
    size_in_bytes_ = nbytes;
    bits_ = new unsigned char[size_in_bytes_];
    std::memcpy(bits_, bits, sizeof(unsigned char) * nbytes);
}

BinaryDescriptor::BinaryDescriptor(const cv::Mat& desc)
{
    if (desc.type() != cv::MAT_8U) {
        throw std::invalid_argument("Input matrix must be of type CV_8U");
    }

    size_in_bytes_ = static_cast<unsigned>(desc.cols);
    size_in_bits_ = size_in_bytes_ * 8;
    bits_ = new unsigned char[size_in_bytes_];

    // Creating the descriptor
    const unsigned char* chars = desc.ptr<unsigned char>(0);
    std::memcpy(bits_, chars, sizeof(unsigned char) * size_in_bytes_);
}

BinaryDescriptor::BinaryDescriptor(const BinaryDescriptor& bd)
    : size_in_bytes_(bd.size_in_bytes_)
    , size_in_bits_(bd.size_in_bits_)
{
    bits_ = new unsigned char[size_in_bytes_];
    std::memcpy(bits_, bd.bits_, sizeof(unsigned char) * size_in_bytes_);
}

// 2. Binary Tree Node
class BinaryTreeNode;
using BinaryTreeNodePtr = std::shared_ptr<BinaryTreeNode>;
class BinaryTreeNode {
public:
    // Constructors
    BinaryTreeNode();
    explicit BinaryTreeNode(const bool leaf,
        BinaryDescriptorPtr desc = nullptr,
        BinaryTreeNodePtr root = nullptr);

    // Methods
    inline bool isLeaf()
    {
        return is_leaf_;
    }

    inline void setLeaf(const bool leaf)
    {
        is_leaf_ = leaf;
    }

    inline bool isBad()
    {
        return is_bad_;
    }

    inline void setBad(const bool bad)
    {
        is_bad_ = bad;
    }

    inline BinaryDescriptorPtr getDescriptor()
    {
        return desc_;
    }

    inline void setDescriptor(BinaryDescriptorPtr desc)
    {
        desc_ = desc;
    }

    inline BinaryTreeNodePtr getRoot()
    {
        return root_;
    }

    inline void setRoot(BinaryTreeNodePtr root)
    {
        root_ = root;
    }

    inline double distance(BinaryDescriptorPtr desc)
    {
        return obindex2::BinaryDescriptor::distHamming(*desc_, *desc);
    }

    inline void addChildNode(BinaryTreeNodePtr child)
    {
        ch_nodes_.insert(child);
    }

    inline void deleteChildNode(BinaryTreeNodePtr child)
    {
        ch_nodes_.erase(child);
    }

    inline std::unordered_set<BinaryTreeNodePtr>* getChildrenNodes()
    {
        return &ch_nodes_;
    }

    inline unsigned childNodeSize()
    {
        return ch_nodes_.size();
    }

    inline void addChildDescriptor(BinaryDescriptorPtr child)
    {
        ch_descs_.insert(child);
    }

    inline void deleteChildDescriptor(BinaryDescriptorPtr child)
    {
        ch_descs_.erase(child);
    }

    inline std::unordered_set<BinaryDescriptorPtr>* getChildrenDescriptors()
    {
        return &ch_descs_;
    }

    inline unsigned childDescriptorSize()
    {
        return ch_descs_.size();
    }

    inline void selectNewCenter()
    {
        desc_ = *std::next(ch_descs_.begin(), std::rand() % ch_descs_.size());
    }

private:
    bool is_leaf_;
    bool is_bad_;
    BinaryDescriptorPtr desc_;
    BinaryTreeNodePtr root_;
    std::unordered_set<BinaryTreeNodePtr> ch_nodes_;
    std::unordered_set<BinaryDescriptorPtr> ch_descs_;
};

using NodeSet = std::unordered_set<BinaryTreeNodePtr>;
BinaryTreeNode::BinaryTreeNode()
    : is_leaf_(false)
    , is_bad_(false)
    , desc_(nullptr)
    , root_(nullptr)
{
}

BinaryTreeNode::BinaryTreeNode(const bool leaf,
    BinaryDescriptorPtr desc,
    BinaryTreeNodePtr root)
    : is_leaf_(leaf)
    , is_bad_(false)
    , desc_(desc)
    , root_(root)
{
}

// 3. Node Queue
struct NodeQueueItem {
public:
    inline explicit NodeQueueItem(const double d,
        const unsigned id,
        BinaryTreeNodePtr n)
        : dist(d)
        , tree_id(id)
        , node(n)
    {
    }

    double dist;
    unsigned tree_id;
    BinaryTreeNodePtr node;

    inline bool operator<(const NodeQueueItem& item) const
    {
        return dist < item.dist;
    }
};

class NodeQueue {
public:
    inline void push(const NodeQueueItem& item)
    {
        items.push_back(item);
    }

    inline NodeQueueItem get(unsigned index)
    {
        return items[index];
    }

    inline void sort()
    {
        std::sort(items.begin(), items.end());
    }

    inline unsigned size()
    {
        return items.size();
    }

private:
    std::vector<NodeQueueItem> items;
};

using NodeQueuePtr = std::shared_ptr<NodeQueue>;

class CompareNodeQueueItem {
public:
    inline bool operator()(const NodeQueueItem& a, const NodeQueueItem& b)
    {
        return a.dist > b.dist;
    }
};

using NodePriorityQueue = std::priority_queue<NodeQueueItem,
    std::vector<NodeQueueItem>,
    CompareNodeQueueItem>;
using NodePriorityQueuePtr = std::shared_ptr<NodePriorityQueue>;

struct DescriptorQueueItem {
public:
    inline explicit DescriptorQueueItem(const double d, BinaryDescriptorPtr bd)
        : dist(d)
        , desc(bd)
    {
    }

    double dist;
    BinaryDescriptorPtr desc;

    inline bool operator<(const DescriptorQueueItem& item) const
    {
        return dist < item.dist;
    }
};

class DescriptorQueue {
public:
    inline void push(const DescriptorQueueItem& item)
    {
        items.push_back(item);
    }

    inline DescriptorQueueItem get(unsigned index)
    {
        return items[index];
    }

    inline void sort()
    {
        std::sort(items.begin(), items.end());
    }

    inline unsigned size()
    {
        return items.size();
    }

private:
    std::vector<DescriptorQueueItem> items;
};
using DescriptorQueuePtr = std::shared_ptr<DescriptorQueue>;

// 4. Binary Tree
class BinaryTree {
public:
    // Constructors
    explicit BinaryTree(BinaryDescriptorSetPtr dset,
        const unsigned tree_id = 0,
        const unsigned k = 16,
        const unsigned s = 150);
    virtual ~BinaryTree();

    // Methods
    void buildTree();
    void deleteTree();
    unsigned traverseFromRoot(BinaryDescriptorPtr q,
        NodeQueuePtr pq,
        DescriptorQueuePtr r);
    void traverseFromNode(BinaryDescriptorPtr q,
        BinaryTreeNodePtr n,
        NodeQueuePtr pq,
        DescriptorQueuePtr r);
    BinaryTreeNodePtr searchFromRoot(BinaryDescriptorPtr q);
    BinaryTreeNodePtr searchFromNode(BinaryDescriptorPtr q,
        BinaryTreeNodePtr n);
    void addDescriptor(BinaryDescriptorPtr q);
    void deleteDescriptor(BinaryDescriptorPtr q);
    void printTree();
    inline unsigned numDegradedNodes()
    {
        return degraded_nodes_;
    }

    inline unsigned numNodes()
    {
        return nset_.size();
    }

private:
    BinaryDescriptorSetPtr dset_;
    unsigned tree_id_;
    BinaryTreeNodePtr root_;
    unsigned k_;
    unsigned s_;
    unsigned k_2_;
    NodeSet nset_;
    std::unordered_map<BinaryDescriptorPtr, BinaryTreeNodePtr> desc_to_node_;

    // Tree statistics
    unsigned degraded_nodes_;
    unsigned nvisited_nodes_;

    void buildNode(BinaryDescriptorSet d, BinaryTreeNodePtr root);
    void printNode(BinaryTreeNodePtr n);
    void deleteNodeRecursive(BinaryTreeNodePtr n);
};
using BinaryTreePtr = std::shared_ptr<BinaryTree>;

BinaryTree::BinaryTree(BinaryDescriptorSetPtr dset,
    const unsigned tree_id,
    const unsigned k,
    const unsigned s)
    : dset_(dset)
    , tree_id_(tree_id)
    , root_(nullptr)
    , k_(k)
    , s_(s)
    , k_2_(k_ / 2)
{
    std::srand(std::time(nullptr));
    buildTree();
}

BinaryTree::~BinaryTree()
{
    deleteTree();
}

void BinaryTree::buildTree()
{
    // Deleting the previous tree, if exists any
    deleteTree();

    degraded_nodes_ = 0;
    nvisited_nodes_ = 0;

    // Creating the root node
    root_ = std::make_shared<BinaryTreeNode>();
    nset_.insert(root_);

    // Generating a new set with the descriptor's ids
    BinaryDescriptorSet descs = *dset_;

    buildNode(descs, root_);
}

void BinaryTree::buildNode(BinaryDescriptorSet dset, BinaryTreeNodePtr root)
{
    // Validate if this should be a leaf node
    if (dset.size() < s_) {
        // We set the previous node as a leaf
        root->setLeaf(true);

        // Adding descriptors as leaf nodes
        for (auto it = dset.begin(); it != dset.end(); it++) {
            BinaryDescriptorPtr d = *it;
            root->addChildDescriptor(d);

            // Storing the reference of the node where the descriptor hangs
            desc_to_node_[d] = root;
        }
    } else {
        // This node should be split
        // Randomly selecting the new centers
        std::vector<BinaryDescriptorPtr> new_centers;
        std::vector<BinaryDescriptorSet> assoc_descs(k_);

        for (unsigned i = 0; i < k_; i++) {
            // Selecting a new center
            BinaryDescriptorPtr desc = *std::next(dset.begin(),
                std::rand() % dset.size());
            new_centers.push_back(desc);
            assoc_descs[i].insert(desc);
            dset.erase(desc);
        }

        // Associating the remaining descriptors to the new centers
        for (auto it = dset.begin(); it != dset.end(); it++) {
            BinaryDescriptorPtr d = *it;
            int best_center = -1;
            double min_dist = std::numeric_limits<double>::max();
            for (unsigned i = 0; i < k_; i++) {
                double dist = obindex2::BinaryDescriptor::distHamming(*d,
                    *(new_centers[i]));

                if (dist < min_dist) {
                    min_dist = dist;
                    best_center = i;
                }
            }

            if (best_center == -1) {
                throw std::runtime_error("No best center found");
            }
            assoc_descs[best_center].insert(d);
        }
        dset.clear();

        // Creating a new tree node for each new cluster
        for (unsigned i = 0; i < k_; i++) {
            BinaryTreeNodePtr node = std::make_shared<BinaryTreeNode>(
                false,
                new_centers[i],
                root);

            // Linking this node with its root
            root->addChildNode(node);

            // Storing the reference to this node
            nset_.insert(node);

            // Recursively apply the algorithm
            buildNode(assoc_descs[i], node);
        }
    }
}

void BinaryTree::deleteTree()
{
    if (nset_.size() > 0) {
        nset_.clear();
        desc_to_node_.clear();

        // Invalidating last reference to root
        root_ = nullptr;
    }
}

unsigned BinaryTree::traverseFromRoot(BinaryDescriptorPtr q,
    NodeQueuePtr pq,
    DescriptorQueuePtr r)
{
    nvisited_nodes_ = 0;
    traverseFromNode(q, root_, pq, r);
    return nvisited_nodes_;
}

void BinaryTree::traverseFromNode(BinaryDescriptorPtr q,
    BinaryTreeNodePtr n,
    NodeQueuePtr pq,
    DescriptorQueuePtr r)
{
    nvisited_nodes_++;
    // If its a leaf node, the search ends
    if (n->isLeaf()) {
        // Adding points to R
        std::unordered_set<BinaryDescriptorPtr>* descs = n->getChildrenDescriptors();
        for (auto it = (*descs).begin(); it != (*descs).end(); it++) {
            BinaryDescriptorPtr d = *it;
            double dist = obindex2::BinaryDescriptor::distHamming(*q, *d);
            DescriptorQueueItem item(dist, d);
            r->push(item);
        }
    } else {
        // Search continues
        std::unordered_set<BinaryTreeNodePtr>* nodes = n->getChildrenNodes();
        int best_node = -1;
        double min_dist = std::numeric_limits<double>::max();

        // Computing distances to nodes
        std::vector<NodeQueueItem> items;
        unsigned node_id = 0;
        // Computing distances to nodes
        for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
            BinaryTreeNodePtr bn = *it;
            double dist = bn->distance(q);
            NodeQueueItem item(dist, tree_id_, bn);
            items.push_back(item);

            if (dist < min_dist) {
                min_dist = dist;
                best_node = node_id;
            }

            node_id++;
        }

        if (best_node != -1) {
            throw std::runtime_error("The node is not a leaf");
        }

        // Adding remaining nodes to pq
        for (unsigned i = 0; i < items.size(); i++) {
            // Is it the best node?
            if (i == static_cast<unsigned>(best_node)) {
                continue;
            }

            pq->push(items[i]);
        }

        // Traversing the best node
        traverseFromNode(q, items[best_node].node, pq, r);
    }
}

BinaryTreeNodePtr BinaryTree::searchFromRoot(BinaryDescriptorPtr q)
{
    return searchFromNode(q, root_);
}

BinaryTreeNodePtr BinaryTree::searchFromNode(BinaryDescriptorPtr q,
    BinaryTreeNodePtr n)
{
    // If it's a leaf node, the search ends
    if (n->isLeaf()) {
        // This is the node where this descriptor should be included
        return n;
    } else {
        // Search continues
        std::unordered_set<BinaryTreeNodePtr>* nodes = n->getChildrenNodes();
        int best_node = -1;
        double min_dist = std::numeric_limits<double>::max();

        // Computing distances to nodes
        std::vector<BinaryTreeNodePtr> items;
        // Computing distances to nodes
        for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
            BinaryTreeNodePtr bn = *it;
            items.push_back(bn);
            double dist = bn->distance(q);

            if (dist < min_dist) {
                min_dist = dist;
                best_node = static_cast<int>(items.size()) - 1;
            }
        }

        if (best_node != -1) {
            throw std::runtime_error("The node is not a leaf");
        }

        // Searching in the best node
        return searchFromNode(q, items[best_node]);
    }
}

void BinaryTree::addDescriptor(BinaryDescriptorPtr q)
{
    BinaryTreeNodePtr n = searchFromRoot(q);
    if (!n->isLeaf()) {
        throw std::runtime_error("The node is not a leaf");
    }
    if (n->childDescriptorSize() + 1 < s_) {
        // There is enough space at this node for this descriptor, so we add it
        n->addChildDescriptor(q);
        // Storing the reference of the node where the descriptor hangs
        desc_to_node_[q] = n;
    } else {
        // This node should be split
        n->setLeaf(false);

        // Gathering the current descriptors
        std::unordered_set<BinaryDescriptorPtr>* descs = n->getChildrenDescriptors();
        BinaryDescriptorSet set;
        for (auto it = (*descs).begin(); it != (*descs).end(); it++) {
            BinaryDescriptorPtr d = *it;
            set.insert(d);
        }
        set.insert(q); // Adding the new descritor to the set

        // Rebuilding this node
        buildNode(set, n);
    }
}

void BinaryTree::deleteDescriptor(BinaryDescriptorPtr q)
{
    // We get the node where the descriptor is stored
    BinaryTreeNodePtr node = desc_to_node_[q];
    if (!node->isLeaf()) {
        throw std::runtime_error("The node is not a leaf");
    }

    // We remove q from the node
    node->deleteChildDescriptor(q);

    if (node->childDescriptorSize() > 0) {
        // We select a new center, if required
        if (node->getDescriptor() == q) {
            // Selecting a new center
            node->selectNewCenter();
        }
    } else {
        // Otherwise, we need to remove the node
        BinaryTreeNodePtr parent = node->getRoot();
        parent->deleteChildNode(node);
        nset_.erase(node);

        deleteNodeRecursive(parent);
    }

    desc_to_node_.erase(q);
}

void BinaryTree::deleteNodeRecursive(BinaryTreeNodePtr n)
{
    if (!n->isLeaf()) {
        throw std::runtime_error("The node is not a leaf");
    }
    // Validating if this node is degraded
    if (n->childNodeSize() < k_2_ && !n->isBad()) {
        degraded_nodes_++;
        n->setBad(true);
    }

    if (n->childNodeSize() == 0 && n != root_) {
        // We remove this node
        BinaryTreeNodePtr parent = n->getRoot();
        parent->deleteChildNode(n);
        nset_.erase(n);

        deleteNodeRecursive(parent);
    }
}

void BinaryTree::printTree()
{
    printNode(root_);
}

void BinaryTree::printNode(BinaryTreeNodePtr n)
{
    std::cout << "---" << std::endl;
    std::cout << "Node: " << n << std::endl;
    std::cout << (n->isLeaf() ? "Leaf" : "Node") << std::endl;
    std::cout << "Descriptor: " << n->getDescriptor() << std::endl;
    if (n->isLeaf()) {
        std::cout << "Children descriptors: " << n->childDescriptorSize() << std::endl;
    } else {
        std::cout << "Children nodes: " << n->childNodeSize() << std::endl;
        std::unordered_set<BinaryTreeNodePtr>* nodes = n->getChildrenNodes();
        for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
            printNode(*it);
        }
    }
}

// 5. Image Index
enum MergePolicy {
    MERGE_POLICY_NONE,
    MERGE_POLICY_AND,
    MERGE_POLICY_OR
};

struct InvIndexItem {
    InvIndexItem()
        : image_id(0)
        , pt(0.0f, 0.0f)
        , dist(std::numeric_limits<double>::max())
        , kp_ind(-1)
    {
    }

    InvIndexItem(const int id,
        const cv::Point2f kp,
        const double d,
        const int kp_i = -1)
        : image_id(id)
        , pt(kp)
        , dist(d)
        , kp_ind(kp_i)
    {
    }

    unsigned image_id;
    cv::Point2f pt;
    double dist;
    int kp_ind;
};

struct ImageMatch {
    ImageMatch()
        : image_id(-1)
        , score(0.0)
    {
    }

    explicit ImageMatch(const int id, const double sc = 0.0)
        : image_id(id)
        , score(sc)
    {
    }

    int image_id;
    double score;

    bool operator<(const ImageMatch& lcr) const { return score > lcr.score; }
};

struct PointMatches {
    std::vector<cv::Point2f> query;
    std::vector<cv::Point2f> train;
};

class ImageIndex {
public:
    // Constructors
    explicit ImageIndex(const unsigned k = 16,
        const unsigned s = 150,
        const unsigned t = 4,
        const MergePolicy merge_policy = MERGE_POLICY_NONE,
        const bool purge_descriptors = true,
        const unsigned min_feat_apps = 3);

    // Methods
    void addImage(const unsigned image_id,
        const std::vector<cv::KeyPoint>& kps,
        const cv::Mat& descs);
    void addImage(const unsigned image_id,
        const std::vector<cv::KeyPoint>& kps,
        const cv::Mat& descs,
        const std::vector<cv::DMatch>& matches);
    void searchImages(const cv::Mat& descs,
        const std::vector<cv::DMatch>& gmatches,
        std::vector<ImageMatch>* img_matches,
        bool sort = true);
    void searchDescriptors(const cv::Mat& descs,
        std::vector<std::vector<cv::DMatch>>* matches,
        const unsigned knn = 2,
        const unsigned checks = 32);
    void deleteDescriptor(const unsigned desc_id);
    void getMatchings(const std::vector<cv::KeyPoint>& query_kps,
        const std::vector<cv::DMatch>& matches,
        std::unordered_map<unsigned, PointMatches>* point_matches);
    inline unsigned numImages()
    {
        return nimages_;
    }

    inline unsigned numDescriptors()
    {
        return dset_.size();
    }

    inline void rebuild()
    {
        if (init_) {
            trees_.clear();
            initTrees();
        }
    }

private:
    BinaryDescriptorSet dset_;
    unsigned k_;
    unsigned s_;
    unsigned t_;
    unsigned init_;
    unsigned nimages_;
    unsigned ndesc_;
    MergePolicy merge_policy_;
    bool purge_descriptors_;
    unsigned min_feat_apps_;

    std::vector<BinaryTreePtr> trees_;
    std::unordered_map<BinaryDescriptorPtr,
        std::vector<InvIndexItem>>
        inv_index_;
    std::unordered_map<BinaryDescriptorPtr, unsigned> desc_to_id_;
    std::unordered_map<unsigned, BinaryDescriptorPtr> id_to_desc_;
    std::list<BinaryDescriptorPtr> recently_added_;

    void initTrees();
    void searchDescriptor(BinaryDescriptorPtr q,
        std::vector<BinaryDescriptorPtr>* neigh,
        std::vector<double>* distances,
        unsigned knn = 2,
        unsigned checks = 32);
    void insertDescriptor(BinaryDescriptorPtr q);
    void deleteDescriptor(BinaryDescriptorPtr q);
    void purgeDescriptors(const unsigned curr_img);
};

ImageIndex::ImageIndex(const unsigned k,
    const unsigned s,
    const unsigned t,
    const MergePolicy merge_policy,
    const bool purge_descriptors,
    const unsigned min_feat_apps)
    : k_(k)
    , s_(s)
    , t_(t)
    , init_(false)
    , nimages_(0)
    , ndesc_(0)
    , merge_policy_(merge_policy)
    , purge_descriptors_(purge_descriptors)
    , min_feat_apps_(min_feat_apps)
{
    // Validating the corresponding parameters
    if (k_ > 1) {
        throw std::runtime_error("k must be greater than 1");
    }
    if (k_ < s_) {
        throw std::runtime_error("k must be less than s");
    }
    if (min_feat_apps <= 0) {
        throw std::runtime_error("min_feat_apps must be greater than 0");
    }
}

void ImageIndex::addImage(const unsigned image_id,
    const std::vector<cv::KeyPoint>& kps,
    const cv::Mat& descs)
{
    // Creating the set of BinaryDescriptors
    for (int i = 0; i < descs.rows; i++) {
        // Creating the corresponding descriptor
        cv::Mat desc = descs.row(i);
        BinaryDescriptorPtr d = std::make_shared<BinaryDescriptor>(desc);
        insertDescriptor(d);

        // Creating the inverted index item
        InvIndexItem item;
        item.image_id = image_id;
        item.pt = kps[i].pt;
        item.dist = 0.0;
        item.kp_ind = i;
        inv_index_[d].push_back(item);
    }

    // If the trees are not initialized, we build them
    if (!init_) {
        if (static_cast<int>(k_) < descs.rows) {
            throw std::runtime_error("k must be less than the number of descriptors");
        }
        initTrees();
        init_ = true;
    }

    // Deleting unstable features
    if (purge_descriptors_) {
        purgeDescriptors(image_id);
    }

    nimages_++;
}

void ImageIndex::addImage(const unsigned image_id,
    const std::vector<cv::KeyPoint>& kps,
    const cv::Mat& descs,
    const std::vector<cv::DMatch>& matches)
{
    // --- Adding new features
    // All features
    std::set<int> points;
    for (unsigned feat_ind = 0; feat_ind < kps.size(); feat_ind++) {
        points.insert(feat_ind);
    }

    // Matched features
    std::set<int> matched_points;
    for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++) {
        matched_points.insert(matches[match_ind].queryIdx);
    }

    // Computing the difference
    std::set<int> diff;
    std::set_difference(points.begin(), points.end(),
        matched_points.begin(), matched_points.end(),
        std::inserter(diff, diff.end()));

    // Inserting new features into the index.
    for (auto it = diff.begin(); it != diff.end(); it++) {
        int index = *it;
        cv::Mat desc = descs.row(index);
        BinaryDescriptorPtr d = std::make_shared<BinaryDescriptor>(desc);
        insertDescriptor(d);

        // Creating the inverted index item
        InvIndexItem item;
        item.image_id = image_id;
        item.pt = kps[index].pt;
        item.dist = 0.0;
        item.kp_ind = index;
        inv_index_[d].push_back(item);
    }

    // --- Updating the matched descriptors into the index
    for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++) {
        int qindex = matches[match_ind].queryIdx;
        int tindex = matches[match_ind].trainIdx;

        BinaryDescriptorPtr q_d = std::make_shared<BinaryDescriptor>(descs.row(qindex));
        BinaryDescriptorPtr t_d = id_to_desc_[tindex];

        // Merge and replace according to the merging policy
        if (merge_policy_ == MERGE_POLICY_AND) {
            *t_d &= *q_d;
        } else if (merge_policy_ == MERGE_POLICY_OR) {
            *t_d |= *q_d;
        }

        // Creating the inverted index item
        InvIndexItem item;
        item.image_id = image_id;
        item.pt = kps[qindex].pt;
        item.dist = matches[match_ind].distance;
        item.kp_ind = qindex;
        inv_index_[t_d].push_back(item);
    }

    // Deleting unstable features
    if (purge_descriptors_) {
        purgeDescriptors(image_id);
    }

    nimages_++;
}

void ImageIndex::searchImages(const cv::Mat& descs,
    const std::vector<cv::DMatch>& gmatches,
    std::vector<ImageMatch>* img_matches,
    bool sort)
{
    // Initializing the resulting structure
    img_matches->resize(nimages_);
    for (unsigned i = 0; i < nimages_; i++) {
        img_matches->at(i).image_id = i;
    }

    // Counting the number of each word in the current document
    std::unordered_map<int, int> nwi_map;
    for (unsigned match_index = 0; match_index < gmatches.size(); match_index++) {
        int train_idx = gmatches[match_index].trainIdx;
        // Updating nwi_map, number of occurrences of a word in an image.
        if (nwi_map.count(train_idx)) {
            nwi_map[train_idx]++;
        } else {
            nwi_map[train_idx] = 1;
        }
    }

    // We process all the matchings again to increase the scores
    for (unsigned match_index = 0; match_index < gmatches.size(); match_index++) {
        int train_idx = gmatches[match_index].trainIdx;
        BinaryDescriptorPtr desc = id_to_desc_[train_idx];

        // Computing the TF term
        double tf = static_cast<double>(nwi_map[train_idx]) / descs.rows;

        // Computing the IDF term
        std::unordered_set<unsigned> nw;
        for (unsigned i = 0; i < inv_index_[desc].size(); i++) {
            nw.insert(inv_index_[desc][i].image_id);
        }
        double idf = std::log(static_cast<double>(nimages_) / nw.size());

        // Computing the final TF-IDF weighting term
        double tfidf = tf * idf;

        for (unsigned i = 0; i < inv_index_[desc].size(); i++) {
            int im = inv_index_[desc][i].image_id;
            img_matches->at(im).score += tfidf;
        }
    }

    if (sort) {
        std::sort(img_matches->begin(), img_matches->end());
    }
}

void ImageIndex::initTrees()
{
    // Creating the trees
    BinaryDescriptorSetPtr dset_ptr = std::make_shared<BinaryDescriptorSet>(dset_);

    for (unsigned i = 0; i < t_; i++) {
        BinaryTreePtr tree_ptr = std::make_shared<BinaryTree>(dset_ptr, i, k_, s_);
        trees_.push_back(tree_ptr);
    }
}

void ImageIndex::searchDescriptors(
    const cv::Mat& descs,
    std::vector<std::vector<cv::DMatch>>* matches,
    const unsigned knn,
    const unsigned checks)
{
    matches->clear();
    for (int i = 0; i < descs.rows; i++) {
        // Creating the corresponding descriptor
        cv::Mat desc = descs.row(i);
        BinaryDescriptorPtr d = std::make_shared<BinaryDescriptor>(desc);

        // Searching the descriptor in the index
        std::vector<BinaryDescriptorPtr> neighs;
        std::vector<double> dists;
        searchDescriptor(d, &neighs, &dists, knn, checks);

        // Translating the resulting matches to CV structures
        std::vector<cv::DMatch> des_match;
        for (unsigned j = 0; j < neighs.size(); j++) {
            cv::DMatch match;
            match.queryIdx = i;
            match.trainIdx = static_cast<int>(desc_to_id_[neighs[j]]);
            match.imgIdx = static_cast<int>(inv_index_[neighs[j]][0].image_id);
            match.distance = dists[j];
            des_match.push_back(match);
        }
        matches->push_back(des_match);
    }
}

void ImageIndex::deleteDescriptor(const unsigned desc_id)
{
    BinaryDescriptorPtr d = id_to_desc_[desc_id];
    // Clearing the descriptor
    deleteDescriptor(d);
}

void ImageIndex::searchDescriptor(BinaryDescriptorPtr q,
    std::vector<BinaryDescriptorPtr>* neigh,
    std::vector<double>* distances,
    unsigned knn,
    unsigned checks)
{
    unsigned points_searched = 0;
    NodePriorityQueue pq;
    DescriptorQueue r;

    // Initializing search structures
    std::vector<NodeQueuePtr> pqs;
    std::vector<DescriptorQueuePtr> rs;
    for (unsigned i = 0; i < trees_.size(); i++) {
        NodeQueuePtr tpq = std::make_shared<NodeQueue>();
        pqs.push_back(tpq);

        DescriptorQueuePtr tr = std::make_shared<DescriptorQueue>();
        rs.push_back(tr);
    }

    // Searching in the trees
    //  #pragma omp parallel for
    for (unsigned i = 0; i < trees_.size(); i++) {
        trees_[i]->traverseFromRoot(q, pqs[i], rs[i]);
    }

    //  Gathering results from each individual search
    std::unordered_set<BinaryDescriptorPtr> already_added;
    for (unsigned i = 0; i < trees_.size(); i++) {
        // Obtaining descriptor nodes
        unsigned r_size = rs[i]->size();
        for (unsigned j = 0; j < r_size; j++) {
            DescriptorQueueItem r_item = rs[i]->get(j);
            std::pair<std::unordered_set<BinaryDescriptorPtr>::iterator,
                bool>
                result;
            result = already_added.insert(r_item.desc);
            if (result.second) {
                r.push(r_item);
                points_searched++;
            }
        }
    }

    // Continuing the search if not enough descriptors have been checked
    if (points_searched < checks) {
        // Gathering the next nodes to search
        for (unsigned i = 0; i < trees_.size(); i++) {
            // Obtaining priority queue nodes
            unsigned pq_size = pqs[i]->size();
            for (unsigned j = 0; j < pq_size; j++) {
                pq.push(pqs[i]->get(j));
            }
        }

        NodePriorityQueuePtr pq_ptr = std::make_shared<NodePriorityQueue>(pq);
        while (points_searched < checks && !pq.empty()) {
            // Get the closest node to continue the search
            NodeQueueItem n = pq.top();
            pq.pop();

            // Searching in the node
            NodeQueuePtr tpq = std::make_shared<NodeQueue>();
            DescriptorQueuePtr tr = std::make_shared<DescriptorQueue>();
            trees_[n.tree_id]->traverseFromNode(q, n.node, tpq, tr);

            // Adding new nodes to search to PQ
            for (unsigned i = 0; i < tpq->size(); i++) {
                pq.push(tpq->get(i));
            }

            for (unsigned j = 0; j < tr->size(); j++) {
                DescriptorQueueItem r_item = tr->get(j);
                std::pair<std::unordered_set<BinaryDescriptorPtr>::iterator,
                    bool>
                    result;
                result = already_added.insert(r_item.desc);
                if (result.second) {
                    r.push(r_item);
                    points_searched++;
                }
            }
        }
    }
    r.sort();

    // Returning the required number of descriptors descriptors
    neigh->clear();
    distances->clear();
    unsigned ndescs = std::min(knn, r.size());
    for (unsigned i = 0; i < ndescs; i++) {
        DescriptorQueueItem d = r.get(i);

        neigh->push_back(d.desc);
        distances->push_back(d.dist);
    }
}

void ImageIndex::insertDescriptor(BinaryDescriptorPtr q)
{
    dset_.insert(q);
    desc_to_id_[q] = ndesc_;
    id_to_desc_[ndesc_] = q;
    ndesc_++;
    recently_added_.push_back(q);

    // Indexing the descriptor inside each tree
    if (init_) {
        //  #pragma omp parallel for
        for (unsigned i = 0; i < trees_.size(); i++) {
            trees_[i]->addDescriptor(q);
        }
    }
}

void ImageIndex::deleteDescriptor(BinaryDescriptorPtr q)
{
    // Deleting the descriptor from each tree
    if (init_) {
        //  #pragma omp parallel for
        for (unsigned i = 0; i < trees_.size(); i++) {
            trees_[i]->deleteDescriptor(q);
        }
    }

    dset_.erase(q);
    unsigned desc_id = desc_to_id_[q];
    desc_to_id_.erase(q);
    id_to_desc_.erase(desc_id);
    inv_index_.erase(q);
}

void ImageIndex::getMatchings(
    const std::vector<cv::KeyPoint>& query_kps,
    const std::vector<cv::DMatch>& matches,
    std::unordered_map<unsigned, PointMatches>* point_matches)
{
    for (unsigned i = 0; i < matches.size(); i++) {
        // Getting the query point
        int qid = matches[i].queryIdx;
        cv::Point2f qpoint = query_kps[qid].pt;

        // Processing the train points
        int tid = matches[i].trainIdx;
        BinaryDescriptorPtr desc_ptr = id_to_desc_[static_cast<unsigned>(tid)];
        for (unsigned j = 0; j < inv_index_[desc_ptr].size(); j++) {
            InvIndexItem item = inv_index_[desc_ptr][j];
            unsigned im_id = item.image_id;
            cv::Point2f tpoint = item.pt;

            (*point_matches)[im_id].query.push_back(qpoint);
            (*point_matches)[im_id].train.push_back(tpoint);
        }
    }
}

void ImageIndex::purgeDescriptors(const unsigned curr_img)
{
    auto it = recently_added_.begin();

    while (it != recently_added_.end()) {
        BinaryDescriptorPtr desc = *it;
        // We assess if at least three images have passed since creation
        if ((curr_img - inv_index_[desc][0].image_id) > 1) {
            // If so, we assess if the feature has been seen at least twice
            if (inv_index_[desc].size() < min_feat_apps_) {
                deleteDescriptor(desc);
            }

            it = recently_added_.erase(it);
        } else {
            // This descriptor should be maintained in the list
            it++;
        }
    }
}
}