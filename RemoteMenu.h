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
  virtual const char* str() const = 0;

  void setPos(uint16_t x, uint16_t y) {
    x_ = x;
    y_ = y;
  }
protected:
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

  const char* str() const override {
    return title_.c_str();
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

  void init() override;
  void draw() override;
private:
  String title_;
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

  const char* str() const override {
    return title_;
  }

  void prev() {
    if (selected_ == 0) {
      selected_ = len_ - 1;
    } else {
      selected_--;
    }
    dirty_ = true;
  }

  void next() {
    if (selected_ == len_ - 1) {
      selected_ = 0;
    } else {
      selected_++;
    }
    dirty_ = true;
  }

  void back();

  void init() override;
  void draw() override;
private:
  const char* title_;
  const uint8_t len_;
  bool is_open_;
  uint8_t selected_;
  MenuItem* (&items_)[];
};

class MenuController: public Widget {
public:
  MenuController(): Widget(0,0) {
    menu_idx_ = -1;
  }

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

  void back() {
    if (menu_idx_ >= 0) {
      menu_stack_[menu_idx_]->back();
      dirty_ = true;
    }
  }

  void init() override {}
  void draw() override {
    if (!dirty_) return;
    if (menu_idx_ >= 0) {
      menu_stack_[menu_idx_]->draw();
    }
    dirty_ = false;
  }
private:
  static constexpr uint8_t maxNestedMenus = 5;

  Menu* menu_stack_[maxNestedMenus];
  int8_t menu_idx_;
};

extern MenuController menuController;
