//
// Created by weihao on 23-8-9.
//
#ifndef SSVIO_ORBVOCABULARY_HPP
#define SSVIO_ORBVOCABULARY_HPP

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"


using namespace DBoW2;

template <class TDescriptor, class F>
class Vocabulary : public TemplatedVocabulary<TDescriptor, F> {
public:
    void loadFromTextFile(const std::string& filename);
};

template <class TDescriptor, class F>
void Vocabulary<TDescriptor, F>::loadFromTextFile(const std::string& filename)
{
    std::ifstream ifs;
    ifs.open(filename.c_str());

    if (!ifs) {
        throw std::string("Could not open file: ") + filename;
    }

    auto& m_words = this->m_words;
    auto& m_nodes = this->m_nodes;
    auto& m_k = this->m_k;
    auto& m_L = this->m_L;

    m_words.clear();
    m_nodes.clear();

    std::string s;
    getline(ifs, s);
    std::stringstream ss;
    ss << s;
    ss >> m_k;
    ss >> m_L;
    int n1, n2;
    ss >> n1;
    ss >> n2;

    if (m_k < 0 || 20 < m_k || m_L < 1 || 10 < m_L || n1 < 0 || 5 < n1 || n2 < 0 || 3 < n2) {
        throw std::string("Vocabulary loading failed");
    }

    this->m_scoring = static_cast<ScoringType>(n1);
    this->m_weighting = static_cast<WeightingType>(n2);
    this->createScoringObject();

    const auto expected_nodes = static_cast<int>((std::pow(static_cast<double>(m_k), static_cast<double>(m_L) + 1.0) - 1) / (m_k - 1));
    m_nodes.reserve(expected_nodes);

    m_words.reserve(std::pow(static_cast<double>(m_k), static_cast<double>(m_L) + 1.0));

    m_nodes.resize(1);
    m_nodes.at(0).id = 0;

    while (!ifs.eof()) {
        std::string s_node;
        getline(ifs, s_node);
        if (s_node == "") {
            continue;
        }
        std::stringstream ss_node;
        ss_node << s_node;

        const int n_id = m_nodes.size();
        m_nodes.resize(m_nodes.size() + 1);
        m_nodes.at(n_id).id = n_id;

        int p_id;
        ss_node >> p_id;
        m_nodes.at(n_id).parent = p_id;
        m_nodes.at(p_id).children.push_back(n_id);

        int is_leaf;
        ss_node >> is_leaf;

        std::stringstream ss_desc;
        for (int i = 0; i < F::L; ++i) {
            std::string s_desc;
            ss_node >> s_desc;
            ss_desc << s_desc << " ";
        }
        F::fromString(m_nodes.at(n_id).descriptor, ss_desc.str());

        ss_node >> m_nodes.at(n_id).weight;

        if (static_cast<bool>(is_leaf)) {
            const int w_id = m_words.size();
            m_words.resize(w_id + 1);

            m_nodes.at(n_id).word_id = w_id;
            m_words.at(w_id) = &m_nodes.at(n_id);
        } else {
            m_nodes.at(n_id).children.reserve(m_k);
        }
    }
}

using ORBVocabulary = Vocabulary<FORB::TDescriptor, FORB>;

#endif // SSVIO_ORBVOCABULARY_HPP
