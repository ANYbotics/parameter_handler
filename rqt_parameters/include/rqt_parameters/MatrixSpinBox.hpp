/*
 * MatrixSpinBox.hpp
 *
 *  Created on: Nov 30, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Qt
#include <QWidget>
#include <QGridLayout>
#include <QDoubleSpinBox>

// Stl
#include <vector>
#include <memory>

class MatrixSpinBox: public QWidget {
 public:
  MatrixSpinBox(QWidget* parent):
    QWidget(parent),
    grid_(this),
    spinboxes_(),
    rows_(0),
    cols_(0)
  {
    grid_.setContentsMargins(0,0,0,0);
    grid_.setAlignment(Qt::AlignTop);
    grid_.setVerticalSpacing(0);
    grid_.setHorizontalSpacing(0);
  }

  ~MatrixSpinBox() {

  }

  void clearLayout(QLayout* layout)
  {
      while (QLayoutItem* item = layout->takeAt(0))
      {
          if (QLayout* childLayout = item->layout())
              clearLayout(childLayout);
          delete item;
      }
  }

  void init(std::size_t rows, std::size_t cols)
  {
    if(rows != rows_ || cols != cols_)
    {
      // Set rows and cols
      rows_ = rows;
      cols_ = cols;

      // Clear grid without deleting objects
      clearLayout(&grid_);

      // Delete old objects and allocate new size
      spinboxes_.clear();
      spinboxes_.resize(rows_ * cols_);

      // Setup spinboxes
      for(auto & sb : spinboxes_) {
        sb.reset(new QDoubleSpinBox(this));
        sb->setAlignment(Qt::AlignTop);
      }

      // Add them to the grid
      for(int r = 0; r<rows_; ++r) {
        for(int c = 0; c<cols_; ++c) {
          grid_.addWidget(getSpinbox(r,c).get(), r , c, Qt::AlignTop);
        }
      }
    }
  }

  std::unique_ptr<QDoubleSpinBox> & getSpinbox(std::size_t row, std::size_t col)
  {
    return spinboxes_.at(row*cols_ + col);
  }

  std::size_t rows() const { return rows_; }
  std::size_t cols() const { return cols_; }

 private:
  QGridLayout grid_;
  std::vector< std::unique_ptr<QDoubleSpinBox> > spinboxes_;
  std::size_t rows_;
  std::size_t cols_;


};
