/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include "RemoteDisplayWidget.h"
#include <Arduino.h>
#include <stdint.h>

class MenuItem: public Widget {
public:
  MenuItem(uint16_t x = 0, uint16_t y = 0, uint16_t w = 0, uint16_t h = 0)
    : Widget(x, y), w_(w), h_(h) {};

  virtual void select() = 0;

  virtual void setMenuPos(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    x_ = x;
    y_ = y;
    w_ = w;
    h_ = h;
  }

  void setSelectState(bool v) {
    selected_ = v;
    dirty_ = true;
  }
protected:
  bool selected_;
  uint16_t w_;
  uint16_t h_;
};

class SimpleMenuItem: public MenuItem {
public:
  typedef void (*Callback)(SimpleMenuItem* self);

  SimpleMenuItem(const char* title, Callback cb)
    : title_(title), cb_(cb) {}

  void select() override {
    if (cb_) {
        cb_(this);
    }
  }

  void setTitle(const char* value) {
    if (title_ != value) {
      title_ = value;
      dirty_ = true;
    }
  }

  void setTitle(const String& value) {
    if (title_ != value) {
      title_ = value;
      dirty_ = true;
    }
  }

  void init() override {}
  void draw() override;
private:
  String title_;
  const Callback cb_;
};

class ToggleMenuItem: public MenuItem {
public:
  typedef void (*Callback)(ToggleMenuItem* self, bool oldState);

  ToggleMenuItem(const char* title, Callback cb)
    : title_(title), cb_(cb) {
  }

  void select() override {
    if (cb_) {
        cb_(this, state_);
    } else {
      toggle();
    }
  }

  void setTitle(const char* value) {
    if (title_ != value) {
      title_ = value;
      dirty_ = true;
    }
  }

  void setTitle(const String& value) {
    if (title_ != value) {
      title_ = value;
      dirty_ = true;
    }
  }

  void toggle() {
    state_ = !state_;
    dirty_ = true;
  }

  void init() override {}
  void draw() override;
private:
  String title_;
  bool state_;
  const Callback cb_;
};

class Menu: public MenuItem {
public:
  Menu(
    const char* title,
    MenuItem* (&items)[],
    uint8_t len,
    uint16_t x,
    uint16_t y,
    uint16_t w,
    uint16_t h)
    : MenuItem(x, y, w, h),
      title_(title),
      len_(len),
      is_open_(false),
      items_(items) {}

  void select() override;

  void setMenuPos(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override {
    closed_x_ = x;
    closed_y_ = y;
    closed_w_ = w;
    closed_h_ = h;
  }

  void prev() {
    items_[selected_idx_]->setSelectState(false);
    if (selected_idx_ == 0) {
      selected_idx_ = len_ - 1;
    } else {
      selected_idx_--;
    }
    items_[selected_idx_]->setSelectState(true);
  }

  void next() {
    items_[selected_idx_]->setSelectState(false);
    if (selected_idx_ == len_ - 1) {
      selected_idx_ = 0;
    } else {
      selected_idx_++;
    }
    items_[selected_idx_]->setSelectState(true);
  }

  void back();

  void init() override;
  void draw() override;
  bool dirty() const override;
  void redraw() override {
    dirty_ = true;
    for(uint8_t idx = 0; idx < len_; idx++) {
      items_[idx]->redraw();
    }
  }
private:
  const char* title_;
  const uint8_t len_;
  bool is_open_;
  uint8_t selected_idx_;
  MenuItem* (&items_)[];
  uint16_t closed_x_;
  uint16_t closed_y_;
  uint16_t closed_w_;
  uint16_t closed_h_;
};

class MenuController: public Widget {
public:
  MenuController(): Widget(0,0), menu_idx_(-1) {}

  void pushMenu(Menu* m);
  void popMenu();

  void prev() {
    if (menu_idx_ >= 0) {
      menu_stack_[menu_idx_]->prev();
      dirty_ = true;
    }
  }

  void next() {
    if (menu_idx_ >= 0) {
      menu_stack_[menu_idx_]->next();
      dirty_ = true;
    }
  }

  void select();
  void back();

  void init() override;
  void draw() override;
  bool dirty() const override {
    if (dirty_) return true;
    if (menu_idx_ >= 0) {
      return menu_stack_[menu_idx_]->dirty();
    }
    return false;
  }
  void redraw() override {
    if (menu_idx_ >= 0) {
      menu_stack_[menu_idx_]->redraw();
    }
    dirty_ = true;
  }
private:
  static constexpr uint8_t maxNestedMenus = 5;

  Menu* menu_stack_[maxNestedMenus];
  int8_t menu_idx_;
};
