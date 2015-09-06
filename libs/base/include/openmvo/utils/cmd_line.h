#ifndef OPENMVO_UTILS_CMD_LINE_H_
#define OPENMVO_UTILS_CMD_LINE_H_

#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <cassert>

namespace mvo{
	/**	选项的基类
	 */
	class Option {

	public:
		/**	构造函数，表示输入选项的词汇和字符
		 */
		Option(char d, std::string name)
			: c(d), used(false), long_name(name) {}
		virtual ~Option() {}
		/**	对argv[0]中的选项进行检查
		 */
		virtual bool check(int& argc, char* argv[]) = 0;
		virtual Option* clone() const = 0;

	public:
		char c; //!< 选项的代表字符 (例如 's'表示 选项 -s)
		bool used; //!< 命令行中是否使用了这个选项
		std::string long_name; //!< 可选的代表选项词汇 (例如 "switch" 表示 --switch)

	};

	/**	表示选项的选择开关
	 */
	class OptionSwitch : public Option {
	public:
		/**	构造函数，代表选项，长字符名可选
		 */
		OptionSwitch(char c, std::string name = "")
			: Option(c, name) {}
		//对argv[0]进行分析
		bool check(int& argc, char* argv[]) {
			if (std::string("-") + c == argv[0] ||
				(!long_name.empty() && std::string("--") + long_name == argv[0])) {
				used = true;
				std::rotate(argv, argv + 1, argv + argc);
				argc -= 1;
				return true;
			}
			else if (std::string(argv[0]).find(std::string("-") + c) == 0) {
				used = true; // 在单一选项中处理多个开关
				std::rotate(argv[0] + 1, argv[0] + 2,
					argv[0] + std::string(argv[0]).size() + 1);
				return true;
			}
			return false;
		}
		/// Copy
		Option* clone() const {
			return new OptionSwitch(c, long_name);
		}
	};

	/**	带有类型T的选项，这个类型可以通过operator>>读取
	 */
	template <class T>
	class OptionField : public Option {
	public:

		/**	构造函数，对应的选项值存储在变量@field
		 */
		OptionField(char c, T& field, std::string name = "")
			: Option(c, name), m_field(field) {}

		/**	根据argv[0] 获得选项和argv[1]获得值，如果值不能读取抛出异常
		 */
		bool check(int& argc, char* argv[]) {
			std::string param; int arg = 0;
			if (std::string("-") + c == argv[0] ||
				(!long_name.empty() && std::string("--") + long_name == argv[0])) {
				if (argc <= 1)
					throw std::string("Option ")
					+ argv[0] + " requires argument";
				param = argv[1]; arg = 2;
			}
			else if (std::string(argv[0]).find(std::string("-") + c) == 0) {
				param = argv[0] + 2; arg = 1;
			}
			else if (!long_name.empty() &&
				std::string(argv[0]).find(std::string("--") + long_name + '=') == 0){
				size_t size = (std::string("--") + long_name + '=').size();
				param = std::string(argv[0]).substr(size); arg = 1;
			}
			if (arg > 0) {
				if (!read_param(param))
					throw std::string("Unable to interpret ")
					+ param + " as argument of " + argv[0];
				used = true;
				std::rotate(argv, argv + arg, argv + argc);
				argc -= arg;
				return true;
			}
			return false;
		}
		/**	读取参数，可以通过流获取，以空格划分
		 */
		bool read_param(const std::string& param) {
			std::stringstream str(param);
			return !((str >> m_field).fail() || !str.eof());
		}
		/// Copy
		Option* clone() const {
			return new OptionField<T>(c, m_field, long_name);
		}
	private:
		T& m_field; //!<用于存储通过argv中获取的值的变量的引用
	};

	/**	模板专有方法，可以获取空格参数
	 */
	template <>
	inline bool OptionField<std::string>::read_param(const std::string& param) {
		m_field = param;
		return true;
	}

	/**	新建一个新的开关选项
	 */
	OptionSwitch make_switch(char c, std::string name = "") {
		return OptionSwitch(c, name);
	}

	/**	新建一个带值的选项
	 */
	template <class T>
	OptionField<T> make_option(char c, T& field, std::string name = "") {
		return OptionField<T>(c, field, name);
	}

	/**	命令行解析
	 */
	class CmdLine {
		std::vector<Option*> opts;
	public:
		/**	析构函数
		 */
		~CmdLine() {
			std::vector<Option*>::iterator it = opts.begin();
			for (; it != opts.end(); ++it)
				delete *it;
		}
		/**	添加一个选项
		 */
		void add(const Option& opt) {
			opts.push_back(opt.clone());
		}

		/**	命令行的解析类似一个过滤器，几乎所有的选项从命令行中移除
		 */
		void process(int& argc, char* argv[]) throw(std::string) {
			std::vector<Option*>::iterator it = opts.begin();
			for (; it != opts.end(); ++it)
				(*it)->used = false;
			for (int i = 1; i < argc;) {
				if (std::string("--") == argv[i]) { // "--" 表示停止选项解析
					std::rotate(argv + i, argv + i + 1, argv + argc);
					--argc;
					break;
				}
				bool found = false; // 发现选项
				for (it = opts.begin(); it != opts.end(); ++it) {
					int n = argc - i;
					found = (*it)->check(n, argv + i);
					if (found) {
						argc = n + i;
						break;
					}
				}
				if (!found) { // 一个单一的连接号和负号都不表示为选项
					if (std::string(argv[i]).size() > 1 && argv[i][0] == '-') {
						std::istringstream str(argv[i]);
						float v;
						if (!(str >> v).eof())
							throw std::string("Unrecognized option ") + argv[i];
					}
					++i;
				}
			}
		}
		/**	是否在解析中使用该字符作为选项
		 */
		bool used(char c) const {
			std::vector<Option*>::const_iterator it = opts.begin();
			for (; it != opts.end(); ++it)
				if ((*it)->c == c)
					return (*it)->used;
			assert(false); // 调用不存在的选项assert
			return false;
		}
	};
}
#endif // OPENMVO_UTILS_CMD_LINE_H_
