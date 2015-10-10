#ifndef OPENMVO_UTILS_FILE_READER_H_
#define OPENMVO_UTILS_FILE_READER_H_

#include <fstream>
#include <vector>

namespace mvo
{

	/**
	 *   实体类的读取，实体类必须含有以下操作
	 *   std::istream& operator >>(std::istream&, Entry&);
	 */
	template<class Entry>
	class FileReader
	{
	public:
		FileReader(const std::string& file) :
			has_entry_(false),
			file_(file),
			file_stream_(file.c_str())
		{}

		virtual ~FileReader()
		{
			file_stream_.close();
		}

		void skip(int num_lines)
		{
			for (int idx = 0; idx < num_lines; ++idx)
			{
				if (!file_stream_.good())  continue;
				file_stream_.ignore(1024, '\n');
				assert(file_stream_.gcount() < 1024);
			}
		}

		void skipComments()
		{
			while (file_stream_.good() && file_stream_.peek() == '#')
				skip(1);
		}

		/// 读取下一个实体类
		bool next()
		{
			if (file_stream_.good() && !file_stream_.eof())
			{
				file_stream_ >> entry_;
				has_entry_ = true;
				return true;
			}
			return false;
		}

		/// 一次性读取所有的实体类
		void readAllEntries(std::vector<Entry>& entries)
		{
			if (!hasEntry()) next();
			do
				entries.push_back(entry());
			while (next());
		}

		/// 得到当前的实体类
		const Entry& entry() const { return entry_; }
		Entry& entry()  { return entry_; }

		/// 确定第一个实体类是否已经被读取
		const bool& hasEntry() const { return has_entry_; }

	private:
		bool has_entry_;
		std::string file_;
		std::ifstream file_stream_;
		Entry entry_;
	};

} // end namespace mvo


#endif // OPENMVO_UTILS_FILE_READER_H_
