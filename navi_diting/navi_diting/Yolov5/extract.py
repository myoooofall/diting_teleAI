import re  
from parameter import *



# 你的输入句子  
# documents = [  
#     "这是一个示例句子。",  
#     "关键词提取是自然语言处理中的一个任务。",  
#     "我想从我的文本中提取关键词。"  
# ]  

def extract(documents, keywords):  
    extracted_keywords = []  # 用于存储提取的关键词  
    for doc in documents:  
        # 利用正则表达式提取关键词，忽略大小写  
        doc_keywords = [word for word in keywords if re.search(word, doc)]  
        extracted_keywords.append(doc_keywords)  
    return extracted_keywords  

# 提取关键词  
# keywords_found = extract_keywords(documents, keyword_list)  

# # 输出结果  
# for i, doc in enumerate(documents):  
#     print(f"句子: {doc}")  
#     print(f"提取的关键词: {keywords_found[i]}")