// print_helper.h
#pragma once
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include <iomanip>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rb::print {

namespace internal {
struct _set_prefix {
  std::string s;
};

struct _add_prefix {
  std::string s;
};
}  // namespace internal

namespace py = pybind11;

enum class Style { Repr = 0, Str = 1 };
enum class ArrayMode { Numpy = 0, Preview = 1, Full = 2, List = 3 };
enum class FloatStyle { Auto = 0, Default = 1, Fixed = 2, Scientific = 3 };
enum class SignStyle { Auto = 0, Minus = 1, Plus = 2, Space = 3 };

struct Options {
  ArrayMode array_mode = ArrayMode::Numpy;
  int linewidth = 120;
  int precision = 3;
  int edgeitems = 3;
  int threshold = 12;
  bool suppress_small = true;

  FloatStyle float_style = FloatStyle::Default;
  SignStyle sign = SignStyle::Minus;
  bool trim_trailing_zeros = false;

  bool multiline_repr = true;
};

/* Forward declarations */
inline std::string inline_obj(py::handle obj);
template <typename T>
inline std::string inline_obj(const T& obj);

inline std::string inline_obj_one_line(py::handle obj);
template <typename T>
inline std::string inline_obj_one_line(const T& obj);

const Options& current_options();

void set_global_options(const Options& o);

void set_printoptions(std::optional<std::string> array_mode,  // "numpy|preview|full"
                      std::optional<int> linewidth, std::optional<int> precision, std::optional<int> edgeitems,
                      std::optional<int> threshold, std::optional<bool> suppress_small,
                      std::optional<bool> multiline_repr,
                      std::optional<std::string> float_style,  // "auto|default|fixed|scientific"
                      std::optional<std::string> sign,         // "auto|minus|plus|space"
                      std::optional<bool> trim_trailing_zeros,
                      std::optional<std::string> scope /* "thread" | "global" (Default: thread) */);

struct PrintConfigGuard {
  std::optional<std::string> array_mode_;
  std::optional<int> linewidth_;
  std::optional<int> precision_;
  std::optional<int> edgeitems_;
  std::optional<int> threshold_;
  std::optional<bool> suppress_small_;
  std::optional<bool> multiline_repr_;
  std::optional<std::string> float_style_;
  std::optional<std::string> sign_;
  std::optional<bool> trim_trailing_zeros_;

  bool active_ = false;

  explicit PrintConfigGuard(std::optional<std::string> array_mode = std::nullopt,
                            std::optional<int> linewidth = std::nullopt, std::optional<int> precision = std::nullopt,
                            std::optional<int> edgeitems = std::nullopt, std::optional<int> threshold = std::nullopt,
                            std::optional<bool> suppress_small = std::nullopt,
                            std::optional<bool> multiline_repr = std::nullopt,
                            std::optional<std::string> float_style = std::nullopt,
                            std::optional<std::string> sign = std::nullopt,
                            std::optional<bool> trim_trailing_zeros = std::nullopt);
  void enter();
  void exit();

  void enter(std::optional<std::string> array_mode, std::optional<int> linewidth, std::optional<int> precision,
             std::optional<int> edgeitems, std::optional<int> threshold, std::optional<bool> suppress_small,
             std::optional<bool> multiline_repr, std::optional<std::string> float_style,
             std::optional<std::string> sign, std::optional<bool> trim_trailing_zeros);
};

struct ScopedPrintOptions {
  explicit ScopedPrintOptions(std::optional<std::string> mode = std::nullopt,
                              std::optional<int> linewidth = std::nullopt, std::optional<int> precision = std::nullopt,
                              std::optional<int> edgeitems = std::nullopt, std::optional<int> threshold = std::nullopt,
                              std::optional<bool> suppress_small = std::nullopt,
                              std::optional<bool> multiline_repr = std::nullopt,
                              std::optional<std::string> float_style = std::nullopt,
                              std::optional<std::string> sign = std::nullopt,
                              std::optional<bool> trim_trailing_zeros = std::nullopt)
      : guard_(mode, linewidth, precision, edgeitems, threshold, suppress_small, multiline_repr, float_style, sign,
               trim_trailing_zeros) {
    guard_.enter();
  }

  ~ScopedPrintOptions() { guard_.exit(); }

 private:
  PrintConfigGuard guard_;
};

std::string np_array_to_string(py::handle arr, Style style);

std::string indent_continuation(const std::string& s, int spaces);

template <typename T>
inline std::string np_shape_dtype(const T& arr_like) {
  py::object arr;
  if constexpr (std::is_same_v<T, py::object>) {
    arr = arr_like;
  } else {
    arr = py::cast(arr_like);
  }
  auto shape = arr.attr("shape");
  auto dtype = arr.attr("dtype").attr("name").cast<std::string>();
  return "array(shape=" + py::repr(shape).cast<std::string>() + ", dtype=" + dtype + ")";
}

inline std::string inline_seq_preview(py::handle seq, char open_ch, char close_ch, bool ml = true) {
  using namespace rb::print;

  const auto& o = current_options();

  py::sequence s = py::reinterpret_borrow<py::sequence>(seq);
  const auto n = static_cast<std::size_t>(py::len(s));

  std::ostringstream out;
  out << open_ch;

  auto emit_item = [&](std::size_t i) {
    // 요소는 한 줄 repr 강제
    if (ml) {
      out << inline_obj(s[i]);
    } else {
      out << inline_obj_one_line(s[i]);
    }
  };

  if (n <= static_cast<std::size_t>(o.threshold)) {
    for (std::size_t i = 0; i < n; ++i) {
      if (i) {
        out << ", ";
      }
      emit_item(i);
    }
  } else {
    const std::size_t edge = static_cast<std::size_t>(std::max(1, std::min(o.edgeitems, static_cast<int>(n / 2))));

    for (std::size_t i = 0; i < edge; ++i) {
      if (i) {
        out << ", ";
      }
      emit_item(i);
    }
    out << ", ...";
    for (std::size_t i = n - edge; i < n; ++i) {
      out << ", ";
      emit_item(i);
    }
  }

  out << close_ch;
  return out.str();
}

inline std::string inline_obj(py::handle obj) {
  if (py::isinstance<py::list>(obj)) {
    return inline_seq_preview(obj, '[', ']', true);
  }
  if (py::isinstance<py::tuple>(obj)) {
    return inline_seq_preview(obj, '(', ')', true);
  }
  return py::repr(obj).cast<std::string>();
}

template <typename T>
inline std::string inline_obj(const T& obj) {
  if constexpr (py::detail::is_pyobject<T>::value) {
    return inline_obj(static_cast<py::handle>(obj));
  } else {
    return inline_obj(py::cast(obj));
  }
}

inline std::string inline_obj_one_line(py::handle obj) {
  ScopedPrintOptions force_one_line(/*mode*/ std::nullopt, /*linewidth*/ std::nullopt, /*precision*/ std::nullopt,
                                    /*edgeitems*/ std::nullopt, /*threshold*/ std::nullopt,
                                    /*suppress_small*/ std::nullopt, /*multiline_repr*/ false,
                                    /*float_style*/ std::nullopt, /*sign*/ std::nullopt,
                                    /*trim_trailing_zeros*/ std::nullopt);
  if (py::isinstance<py::list>(obj)) {
    return inline_seq_preview(obj, '[', ']', false);
  }
  if (py::isinstance<py::tuple>(obj)) {
    return inline_seq_preview(obj, '(', ')', false);
  }
  return py::repr(obj).cast<std::string>();
}

template <typename T>
inline std::string inline_obj_one_line(const T& obj) {
  if constexpr (py::detail::is_pyobject<T>::value) {
    return inline_obj_one_line(static_cast<py::handle>(obj));
  } else {
    return inline_obj_one_line(py::cast(obj));
  }
}

void apply_float_format(std::ostream& os, Style style);

struct StreamFloatGuard {
  explicit StreamFloatGuard(std::ostream& os, Style style);
  ~StreamFloatGuard();

 private:
  std::ostream* os_;
  std::streamsize old_prec_;
  std::ios_base::fmtflags old_flags_;
  bool old_showpos_;
};

template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
inline std::string format_number(T value, Style style) {
  std::ostringstream ss;
  StreamFloatGuard guard(ss, style);
  ss << value;
  std::string s = ss.str();

  const auto& o = current_options();
  if (o.sign == SignStyle::Space && value >= 0) {
    if (s.empty() || (s[0] != '+' && s[0] != ' ')) {
      s.insert(s.begin(), ' ');
    }
  }
  if (o.trim_trailing_zeros) {
    auto pos_e = s.find_first_of("eE");
    std::string mant = (pos_e == std::string::npos) ? s : s.substr(0, pos_e);
    std::string exp = (pos_e == std::string::npos) ? "" : s.substr(pos_e);
    auto pos_dot = mant.find('.');
    if (pos_dot != std::string::npos) {
      while (!mant.empty() && mant.back() == '0') {
        mant.pop_back();
      }
      if (!mant.empty() && mant.back() == '.') {
        mant.pop_back();
      }
    }
    s = mant + exp;
  }
  return s;
}

bool use_multiline_repr();

inline internal::_set_prefix prefix(std::string s) {
  return {std::move(s)};
}

inline internal::_add_prefix add_prefix(std::string s) {
  return {std::move(s)};
}

namespace internal {
class prefixing_buf : public std::streambuf {
 public:
  prefixing_buf() : at_line_start_(true) {}

  std::string str() const { return out_; }

  void clear() {
    out_.clear();
    at_line_start_ = true;
  }

  void set_base_prefix(std::string p) { prefix_ = std::move(p); }

  void add_prefix(std::string p) { prefix_ += std::move(p); }

 protected:
  int_type overflow(int_type ch) override {
    if (traits_type::eq_int_type(ch, traits_type::eof()))
      return traits_type::not_eof(ch);

    if (at_line_start_) {
      out_.append(prefix_);
      at_line_start_ = false;
    }

    out_.push_back(static_cast<char>(ch));
    if (ch == '\n')
      at_line_start_ = true;
    return ch;
  }

  std::streamsize xsputn(const char* s, std::streamsize n) override {
    std::streamsize written = 0;
    for (std::streamsize i = 0; i < n; ++i) {
      overflow(traits_type::to_int_type(s[i]));
      ++written;
    }
    return written;
  }

 private:
  std::string prefix_;
  std::string out_;
  bool at_line_start_;
};

}  // namespace internal

class ReprStream : public std::ostream {
 public:
  ReprStream() : std::ostream(&buf_) {}

  std::string str() const { return buf_.str(); }

  void clear_string() { buf_.clear(); }

  internal::prefixing_buf& rdbuf_ref() { return buf_; }

 private:
  internal::prefixing_buf buf_;
};

inline ReprStream& operator<<(ReprStream& os, const internal::_set_prefix& sp) {
  os.rdbuf_ref().set_base_prefix(sp.s);
  return os;
}

inline ReprStream& operator<<(ReprStream& os, const internal::_add_prefix& ao) {
  os.rdbuf_ref().add_prefix(ao.s);
  return os;
}

inline std::string to_fixed(double v, int prec = 6) {
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os << std::setprecision(prec) << v;
  return os.str();
}

template <size_t N>
inline std::string py_list(const std::array<unsigned int, N>& a) {
  std::ostringstream os;
  os << "[";
  for (size_t i = 0; i < N; ++i) {
    if (i) {
      os << ", ";
    }
    os << a[i];
  }
  os << "]";
  return os.str();
}

template <size_t N>
inline std::string py_list(const std::array<std::string_view, N>& a) {
  std::ostringstream os;
  os << "[";
  for (size_t i = 0; i < N; ++i) {
    if (i)
      os << ", ";
    os << "\'" << a[i] << "\'";
  }
  os << "]";
  return os.str();
}

}  // namespace rb::print
