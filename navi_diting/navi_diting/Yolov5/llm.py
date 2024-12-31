import pandas as pd  
import numpy as np  
import jieba  
import torch  
import torch.nn as nn  
from sklearn.model_selection import train_test_split  
from sklearn.preprocessing import LabelEncoder  
from torch.utils.data import Dataset, DataLoader  
import torch.nn.functional as F  
from parameter import *

df = pd.DataFrame(data)  

# 中文分词  
df['input'] = df['input'].apply(lambda x: " ".join(jieba.cut(x)))  

# 数据准备  
X = df['input']  
y = df['keyword']  

# 编码关键词  
label_encoder = LabelEncoder()  
y_encoded = label_encoder.fit_transform(y)  

# 分割训练集和测试集  
X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42)  

# 创建词汇表  
word_to_idx = {}  
for sentence in X_train:  
    for word in sentence.split():  
        if word not in word_to_idx:  
            word_to_idx[word] = len(word_to_idx) + 1  # 1-based index  

# 定义数据集类  
class TextDataset(Dataset):  
    def __init__(self, texts, labels, word_to_idx):  
        self.texts = texts.reset_index(drop=True)  # reset index  
        self.labels = labels  # reset index  
        self.word_to_idx = word_to_idx  

    def __len__(self):  
        return len(self.texts)  

    def __getitem__(self, idx):  
        text = self.texts[idx]  
        label = self.labels[idx]  
        # 将文本转为索引  
        text_indices = [self.word_to_idx[word] for word in text.split()]  
        return torch.tensor(text_indices, dtype=torch.long), torch.tensor(label, dtype=torch.long)  

# 自定义 Collate 函数  
def collate_fn(batch):  
    texts, labels = zip(*batch)  
    # 使用 pad_sequence 函数进行填充  
    texts_padded = nn.utils.rnn.pad_sequence(texts, batch_first=True, padding_value=0)  
    labels_tensor = torch.stack(labels)  
    return texts_padded, labels_tensor  

# 创建数据集和数据加载器  
train_dataset = TextDataset(X_train, y_train, word_to_idx)  
test_dataset = TextDataset(X_test, y_test, word_to_idx)  

train_loader = DataLoader(train_dataset, batch_size=3, shuffle=True, collate_fn=collate_fn)  
test_loader = DataLoader(test_dataset, batch_size=3, shuffle=False, collate_fn=collate_fn)  

# 定义 LSTM 网络  
class LSTMClassifier(nn.Module):  
    def __init__(self, input_dim, embedding_dim, hidden_dim, output_dim):  
        super(LSTMClassifier, self).__init__()  
        self.embedding = nn.Embedding(input_dim, embedding_dim)  
        self.lstm = nn.LSTM(embedding_dim, hidden_dim, batch_first=True)  
        self.fc = nn.Linear(hidden_dim, output_dim)  

    def forward(self, x):  
        # x 是一个批次的文本索引  
        embedded = self.embedding(x)  
        lstm_out, (hn, cn) = self.lstm(embedded)  
        out = self.fc(hn[-1])  
        return out  

# 初始化模型参数  
input_dim = len(word_to_idx) + 1  # +1 for padding  
embedding_dim = 128  
hidden_dim = 64  
output_dim = len(label_encoder.classes_)  

# 创建模型实例  
model = LSTMClassifier(input_dim, embedding_dim, hidden_dim, output_dim)  
model = model.to(torch.device('cuda' if torch.cuda.is_available() else 'cpu'))  

# 定义损失函数和优化器  
criterion = nn.CrossEntropyLoss()  
optimizer = torch.optim.Adam(model.parameters())  

# 训练模型  
def train_model(model, train_loader, optimizer, criterion):  
    model.train()  
    for texts, labels in train_loader:  
        optimizer.zero_grad()  
        predictions = model(texts)  
        loss = criterion(predictions, labels)  
        loss.backward()  
        optimizer.step()  

# 进行训练  
for epoch in range(40):  
    train_model(model, train_loader, optimizer, criterion)  
    print(f'Epoch {epoch + 1} finished.')  

# 预测函数
def predict(model, sentence):
    model.eval()
    # 将句子分词并转换为词索引
    tokens = [word_to_idx[word] for word in sentence.split() if word in word_to_idx]

    # 如果没有找到有效的词，返回 None 或一个默认值
    if not tokens:
        return "未找到有效关键词"

    # 创建 LongTensor
    tensor = torch.tensor(tokens, dtype=torch.long).unsqueeze(0).to(torch.device('cuda' if torch.cuda.is_available() else 'cpu'))

    with torch.no_grad():
        predictions = model(tensor)

    return label_encoder.inverse_transform([torch.argmax(predictions).item()])

# 测试新的输入
test_sentence = "给我来一瓶矿泉水"
predicted_keyword = predict(model, test_sentence)
print(f"输入: '{test_sentence}' 提取的关键词: '{predicted_keyword[0]}'")