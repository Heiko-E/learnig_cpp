#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
using namespace std;

class HexBoard
/// @brief class to hold information about a hex board

{
public:
    HexBoard() : size_(7)
    {
        /// @brief The default board has a size of 7*7

        this->addEdges();
    }

    HexBoard(const HexBoard &board) : size_(board.size_),
                                      edges_(board.edges_),
                                      player1_(board.player1_),
                                      player2_(board.player2_)
    {
        /// @brief Create a copy of the given board.
        /// @param board The hex board to be copied
    }

    HexBoard(int size) : size_(size)
    {
        /// @brief Create a board with the given size. Board will have size*size fileds
        /// @param size of one board dimension.

        this->addEdges();
    }

    int size() const
    {
        /// @brief get the size of the board
        /// @return number of rows and columns of the board

        return this->size_;
    }

    inline int field(int row, int column) const
    {
        /// @brief get the index of the field
        /// @param row row index
        /// @param column column index
        /// @return index of the field
        if (0 <= row && row < this->size_ && 0 <= column && column < this->size_)
        {
            return row * this->size_ + column;
        }
        return -1;
    }

    vector<int> neighbors(int row, int column) const
    {
        /// @brief lists all fields y such that there is an edge from x to y.
        /// @param row row index of starting field of the edge
        /// @param column column index of starting field of the edge
        /// @return all neighbors of the field
        if (0 <= row && row < this->size_ && 0 <= column && column < this->size_)
        {
            return this->edges_.at(row).at(column);
        }
        return vector<int>();
    }

    bool player1(int row, int column)
    {
        int field = this->field(row, column);
        if (this->isSelected(field))
        {
            return false;
        }
        this->player1_.push_back(field);
        return true;
    }

    bool player2(int row, int column)
    {
        int field = this->field(row, column);
        if (this->isSelected(field))
        {
            return false;
        }
        this->player2_.push_back(field);
        return true;
    }

    friend ostream &operator<<(ostream &os, const HexBoard &board);

private:
    /* Number of rows and columns of the board. */
    int size_;
    /* Edges are stored in a map to enable fast lookup if the starting field is given.
      Key is the start field.*/
    vector<vector<vector<int>>> edges_;
    /* All fields of playrer 1*/
    vector<int> player1_;
    /* All fields of playrer 2*/
    vector<int> player2_;
    /* All fields at the top side of the board*/

    void addEdges()
    {
        int nodes = this->size_ * this->size_;
        for (int column = 0; column < this->size_; column++)
        {
            this->edges_.push_back(vector<vector<int>>(this->size_, vector<int>()));
        }
        for (int row = 0; row < this->size_; row++)
        {
            for (int column = 0; column < this->size_; column++)
            {
                int field = this->field(row, column);
                if (column > 0)
                {
                    this->edges_.at(row).at(column).push_back(this->field(row, column - 1));
                    this->edges_.at(row).at(column - 1).push_back(field);
                }
                if (row > 0)
                {
                    this->edges_.at(row).at(column).push_back(this->field(row - 1, column));
                    this->edges_.at(row - 1).at(column).push_back(field);
                    if ((column + 1) < this->size_)
                    {
                        this->edges_.at(row).at(column).push_back(this->field(row - 1, column + 1));
                        this->edges_.at(row - 1).at(column + 1).push_back(field);
                    }
                }
            }
        }
    }

    bool isSelected(int field) const
    {
        return (find(this->player1_.begin(), this->player1_.end(), field) != this->player1_.end() &&
                find(this->player2_.begin(), this->player2_.end(), field) != this->player2_.end());
    }
};

ostream &operator<<(ostream &os, const HexBoard &board)
{
    /// @brief Output operator for HexBoard class
    /// @param os output stream
    /// @param board the board object to add to the output
    /// @return output stream

    os << "Hex Board:" << endl;
    for (int row = 0; row < board.size(); row++)
    {
        string spacer;
        for (int i = 0; i < row; i++)
        {
            spacer = spacer + "  ";
        }
        os << spacer << '.';
        for (int column = 1; column < board.size(); column++)
        {
            auto neighbors = board.neighbors(row, column);
            if (find(neighbors.begin(), neighbors.end(), board.field(row, column - 1)) != neighbors.end())
            {
                os << " - ";
            }
            else
            {
                os << "   ";
            }
            os << '.';
        }
        os << endl;
        os << spacer << ' ';
        for (int column = 0; column < board.size(); column++)
        {
            int field = board.field(row, column);
            auto neighbors = board.neighbors(row, column);
            if (column > 0)
            {
                if (find(neighbors.begin(), neighbors.end(), board.field(row + 1, column - 1)) != neighbors.end())
                {
                    os << "/ ";
                }
                else
                {
                    os << "  ";
                }
            }
            if (find(neighbors.begin(), neighbors.end(), board.field(row + 1, column)) != neighbors.end())
            {
                os << "\\ ";
            }
            else
            {
                os << "  ";
            }
        }
        os << endl;
    }
    os << endl;
    return os;
}

int main()
{
    HexBoard board(3);
    cout << board << endl;
    return 0;
}