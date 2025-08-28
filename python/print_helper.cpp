#include "print_helper.h"

#include <pybind11/eval.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <limits>
#include <sstream>

namespace rb::print {
namespace py = pybind11;

// ---- internal state ---------------------------------------------------------
static Options g_global;
static thread_local std::vector<Options> t_stack;

// ---- parse helpers ----------------------------------------------------------
static ArrayMode parse_array_mode(const std::string& s) {
  if (s == "numpy") {
    return ArrayMode::Numpy;
  }
  if (s == "preview") {
    return ArrayMode::Preview;
  }
  if (s == "full") {
    return ArrayMode::Full;
  }
  if (s == "list") {
    return ArrayMode::List;
  }
  throw std::runtime_error("array_mode must be 'numpy'|'preview'|'full'|'list'");
}

static FloatStyle parse_float_style(const std::string& s) {
  if (s == "auto") {
    return FloatStyle::Auto;
  }
  if (s == "default") {
    return FloatStyle::Default;
  }
  if (s == "fixed") {
    return FloatStyle::Fixed;
  }
  if (s == "scientific") {
    return FloatStyle::Scientific;
  }
  throw std::runtime_error("float_style must be 'auto'|'default'|'fixed'|'scientific'");
}

static SignStyle parse_sign(const std::string& s) {
  if (s == "auto") {
    return SignStyle::Auto;
  }
  if (s == "minus") {
    return SignStyle::Minus;
  }
  if (s == "plus") {
    return SignStyle::Plus;
  }
  if (s == "space") {
    return SignStyle::Space;
  }
  throw std::runtime_error("sign must be 'auto'|'minus'|'plus'|'space'");
}

// ---- API: options -----------------------------------------------------------
const Options& current_options() {
  if (!t_stack.empty()) {
    return t_stack.back();
  }
  return g_global;
}

void set_global_options(const Options& o) {
  g_global = o;
}

void set_printoptions(std::optional<std::string> array_mode, std::optional<int> linewidth, std::optional<int> precision,
                      std::optional<int> edgeitems, std::optional<int> threshold, std::optional<bool> suppress_small,
                      std::optional<bool> multiline_repr, std::optional<std::string> float_style,
                      std::optional<std::string> sign, std::optional<bool> trim_trailing_zeros,
                      std::optional<std::string> scope) {
  const bool to_global = (scope && *scope == "global");
  Options base = to_global ? g_global : current_options();

  if (array_mode) {
    base.array_mode = parse_array_mode(*array_mode);
  }
  if (linewidth) {
    base.linewidth = *linewidth;
  }
  if (precision) {
    base.precision = *precision;
  }
  if (edgeitems) {
    base.edgeitems = *edgeitems;
  }
  if (threshold) {
    base.threshold = *threshold;
  }
  if (suppress_small) {
    base.suppress_small = *suppress_small;
  }
  if (multiline_repr) {
    base.multiline_repr = *multiline_repr;
  }
  if (float_style) {
    base.float_style = parse_float_style(*float_style);
  }
  if (sign) {
    base.sign = parse_sign(*sign);
  }
  if (trim_trailing_zeros) {
    base.trim_trailing_zeros = *trim_trailing_zeros;
  }

  if (to_global)
    set_global_options(base);
  else {
    if (t_stack.empty()) {
      t_stack.push_back(base);
    } else {
      t_stack.back() = base;
    }
  }
}

PrintConfigGuard::PrintConfigGuard(std::optional<std::string> array_mode, std::optional<int> linewidth,
                                   std::optional<int> precision, std::optional<int> edgeitems,
                                   std::optional<int> threshold, std::optional<bool> suppress_small,
                                   std::optional<bool> multiline_repr, std::optional<std::string> float_style,
                                   std::optional<std::string> sign, std::optional<bool> trim_trailing_zeros)
    : array_mode_(std::move(array_mode)),
      linewidth_(linewidth),
      precision_(precision),
      edgeitems_(edgeitems),
      threshold_(threshold),
      suppress_small_(suppress_small),
      multiline_repr_(multiline_repr),
      float_style_(std::move(float_style)),
      sign_(std::move(sign)),
      trim_trailing_zeros_(trim_trailing_zeros) {}

void PrintConfigGuard::enter() {
  Options base = current_options();
  if (array_mode_) {
    base.array_mode = parse_array_mode(*array_mode_);
  }
  if (linewidth_) {
    base.linewidth = *linewidth_;
  }
  if (precision_) {
    base.precision = *precision_;
  }
  if (edgeitems_) {
    base.edgeitems = *edgeitems_;
  }
  if (threshold_) {
    base.threshold = *threshold_;
  }
  if (suppress_small_) {
    base.suppress_small = *suppress_small_;
  }
  if (multiline_repr_) {
    base.multiline_repr = *multiline_repr_;
  }
  if (float_style_) {
    base.float_style = parse_float_style(*float_style_);
  }
  if (sign_) {
    base.sign = parse_sign(*sign_);
  }
  if (trim_trailing_zeros_) {
    base.trim_trailing_zeros = *trim_trailing_zeros_;
  }

  t_stack.push_back(base);
  active_ = true;
}

void PrintConfigGuard::enter(std::optional<std::string> array_mode, std::optional<int> linewidth,
                             std::optional<int> precision, std::optional<int> edgeitems, std::optional<int> threshold,
                             std::optional<bool> suppress_small, std::optional<bool> multiline_repr,
                             std::optional<std::string> float_style, std::optional<std::string> sign,
                             std::optional<bool> trim_trailing_zeros) {
  array_mode_ = std::move(array_mode);
  linewidth_ = linewidth;
  precision_ = precision;
  edgeitems_ = edgeitems;
  threshold_ = threshold;
  suppress_small_ = suppress_small;
  multiline_repr_ = multiline_repr;
  float_style_ = std::move(float_style);
  sign_ = std::move(sign);
  trim_trailing_zeros_ = trim_trailing_zeros;
  enter();
}

void PrintConfigGuard::exit() {
  if (active_) {
    if (!t_stack.empty()) {
      t_stack.pop_back();
    }
    active_ = false;
  }
}

std::string indent_continuation(const std::string& s, int spaces) {
  if (spaces <= 0) {
    return s;
  }
  std::string pad(spaces, ' ');
  std::ostringstream out;

  bool first = true;
  size_t start = 0;
  for (size_t i = 0; i <= s.size(); ++i) {
    if (i == s.size() || s[i] == '\n') {
      const std::string_view line(&s[start], i - start);
      if (!first) {
        out << pad;
      }
      out << line;
      if (i != s.size()) {
        out << '\n';
      }
      start = i + 1;
      first = false;
    }
  }
  return out.str();
}

void apply_float_format(std::ostream& os, Style /*style*/) {
  const auto& o = current_options();

  // flags
  os.unsetf(std::ios::floatfield);
  switch (o.float_style) {
    case FloatStyle::Default:
      os.setf(std::ios::fmtflags(0), std::ios::floatfield);
      break;  // defaultfloat
    case FloatStyle::Fixed:
      os.setf(std::ios::fixed, std::ios::floatfield);
      break;
    case FloatStyle::Scientific:
      os.setf(std::ios::scientific, std::ios::floatfield);
      break;
    case FloatStyle::Auto: /* leave as-is */
      break;
  }

  os.precision(o.precision);

  if (o.sign == SignStyle::Plus) {
    os.setf(std::ios::showpos);
  } else {
    os.unsetf(std::ios::showpos);
  }
}

StreamFloatGuard::StreamFloatGuard(std::ostream& os, Style style)
    : os_(&os), old_prec_(os.precision()), old_flags_(os.flags()), old_showpos_((os.flags() & std::ios::showpos) != 0) {
  (void)style;
  apply_float_format(os, style);
}

StreamFloatGuard::~StreamFloatGuard() {
  if (os_) {
    os_->precision(old_prec_);
    os_->flags(old_flags_);
    if (old_showpos_) {
      os_->setf(std::ios::showpos);
    } else {
      os_->unsetf(std::ios::showpos);
    }
  }
}

bool use_multiline_repr() {
  return current_options().multiline_repr;
}

static void _dump_list_like(std::ostringstream& out, py::handle obj, Style style) {
  const auto& o = current_options();

  if (py::isinstance<py::list>(obj) || py::isinstance<py::tuple>(obj)) {
    py::sequence seq = py::reinterpret_borrow<py::sequence>(obj);
    auto emit_item = [&](std::size_t i) {
      _dump_list_like(out, seq[i], style);
    };

    out << "[";
    const auto n = static_cast<std::size_t>(py::len(seq));
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
    out << "]";
    return;
  }

  if (py::isinstance<py::float_>(obj)) {
    double v = obj.cast<double>();
    out << format_number(v, style);
  } else if (py::isinstance<py::int_>(obj)) {
    out << obj.cast<long long>();
  } else if (py::isinstance<py::bool_>(obj)) {
    out << (obj.cast<bool>() ? "True" : "False");
  } else {
    out << py::repr(obj).cast<std::string>();
  }
}

// ---- numpy array2string bridge ---------------------------------------------
static std::string call_array2string(py::handle arr, const Options& o, Style style) {
  using namespace py::literals;
  py::object np = py::module_::import("numpy");

  if (o.array_mode == ArrayMode::List) {
    py::object py_list = arr.attr("tolist")();
    std::ostringstream out;
    _dump_list_like(out, py_list, style);
    return out.str();
  }

  if (o.array_mode == ArrayMode::Numpy) {
    return np.attr("array2string")(arr).cast<std::string>();
  }

  py::dict kwargs;

  if (o.array_mode == ArrayMode::Full) {
    kwargs["threshold"] = std::numeric_limits<int>::max();
  } else {
    kwargs["threshold"] = o.threshold;
    kwargs["edgeitems"] = o.edgeitems;
  }

  kwargs["max_line_width"] = o.linewidth;
  kwargs["precision"] = o.precision;
  kwargs["suppress_small"] = o.suppress_small;

  if (o.sign == SignStyle::Plus) {
    kwargs["sign"] = "+";
  } else if (o.sign == SignStyle::Minus) {
    kwargs["sign"] = "-";
  } else if (o.sign == SignStyle::Space) {
    kwargs["sign"] = " ";
  }

  if (o.float_style == FloatStyle::Scientific) {
    std::string code = "lambda x: format(x, '." + std::to_string(o.precision) + "e')";
    py::object fmtfunc = py::eval(code);
    kwargs["formatter"] = py::dict("float_kind"_a = fmtfunc);
  } else {
    if (o.trim_trailing_zeros) {
      kwargs["floatmode"] = "unique";
    } else if (o.float_style == FloatStyle::Fixed) {
      kwargs["floatmode"] = "fixed";
    } else if (o.float_style == FloatStyle::Default) {
      kwargs["floatmode"] = "maxprec_equal";
    }
  }

  return np.attr("array2string")(arr, **kwargs).cast<std::string>();
}

std::string np_array_to_string(py::handle arr, Style style) {
  const auto& o = current_options();
  return call_array2string(arr, o, style);
}

}  // namespace rb::print
