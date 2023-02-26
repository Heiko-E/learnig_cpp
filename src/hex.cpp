#include <iostream>
#include <vector>
#include <array>
#include <map>
using namespace std;

class HexField
/// @brief class to hold information about a hex field

{
public:
    HexField() {}

    friend ostream &operator<<(ostream &os, const HexField &field);

private:
};

ostream &operator<<(ostream &os, const HexField &field)
{
    /// @brief Output operator for TreeNode class
    /// @param os output stream
    /// @param field the field element to add to the output
    /// @return output stream

    return os;
}

class HexBoard
/// @brief class to hold information about a hex board

{
public:
    HexBoard() : fields_(vector<HexField>(7 * 7))
    {
        /// @brief The default board has a size of 7*7
        this->addEdges();
    }

    HexBoard(const HexBoard &board) : fields_(board.fields_),
                                      edges_(board.edges_)
    {
        /// @brief Create a copy of the given board.
        /// @param board The hex board to be copied
    }

    HexBoard(int size) : fields_(vector<HexField>(size * size))
    {
        /// @brief Create a board with the given size. Board will have size*size fileds
        /// @param size of one board dimension.
        this->addEdges();
    }

    int size() const
    {
        /// @brief get the size of the board
        /// @return number of fields of the board

        return this->fields_.size();
    }

    vector<int> neighbors(int x) const
    {
        /// @brief lists all fields y such that there is an edge from x to y.
        /// @param x index of starting field of the edge
        /// @return all neighbors of the field

        vector<int> neighbors;
        if (0 <= x < this->size())
        {
            for (int field : this->edges_.at(x))
            {
                neighbors.push_back(field);
            }
        }
        return neighbors;
    }

    template <class N>
    friend ostream &operator<<(ostream &os, const HexBoard &board);

private:
    /* All fields of the boards. The index in the vectoris used as a reference. */
    vector<HexField> fields_;
    /*Edges are stored in a map to enable fast lookup if the starting field is given.
      Key is the start field.*/
    map<int, vector<int>> edges_;

    void addEdges()
    {
        for (int field = 0; field < this->fields_.size(); field++)
        {
            this->edges_.insert(pair<int, vector<int>>(field, vector<int>()));
        }
    }

    bool addEdge(int x, int y)
    {
        /// @brief adds to the board the edge from x to y, if it is not there.
        /// @param x index of start field of the edge
        /// @param y index of end field of the edge
        /// @return true if the edge was successfully added
        if (0 <= y < this->size() && 0 <= x < this->size())
        {
            this->edges_.at(x).push_back(y);
            this->edges_.at(y).push_back(x);
            return true;
        }
        return false;
    }
};

ostream &operator<<(ostream &os, const HexBoard &board)
{
    /// @brief Output operator for HexBoard class
    /// @param os output stream
    /// @param board the board object to add to the output
    /// @return output stream

    os << "Hex Board:" << endl;

    os << endl;
    return os;
}

int main()
{
    HexBoard board(11);
    cout << board << endl;
    return 0;
}