import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

class DynamicMLP(nn.Module):
    def __init__(self, input_dim, hidden_dims, output_dim):
        super(DynamicMLP, self).__init__()
        self.hidden_dims = hidden_dims.copy()  # Make a copy to modify
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.layers = []
        self._build_layers()

    def _build_layers(self):
        # Build the list of layers
        in_dim = self.input_dim
        for hidden_dim in self.hidden_dims:
            self.layers.append(nn.Linear(in_dim, hidden_dim))
            self.layers.append(nn.ReLU())
            in_dim = hidden_dim
        self.layers.append(nn.Linear(in_dim, self.output_dim))
        # Rebuild the sequential network
        self.network = nn.Sequential(*self.layers)

    def forward(self, x):
        return self.network(x)

    def add_layer(self, hidden_dim):
        # 在现有网络末尾添加一个新的隐藏层
        self.hidden_dims.append(hidden_dim)
        self._build_layers()
        print(f"Added a new hidden layer with {hidden_dim} neurons.")

    def remove_layer(self):
        # 移除最后一个隐藏层
        if len(self.hidden_dims) <= 1:
            print("Cannot remove layer: network has only one hidden layer.")
            return
        self.hidden_dims.pop()
        self._build_layers()
        print("Removed the last hidden layer.")

# 示例数据集（简单的回归任务）
def create_sample_data(num_samples=1000, input_dim=5, noise=0.1):
    X = torch.randn(num_samples, input_dim)
    true_weights = torch.tensor([2.0, -3.0, 1.0, 0.5, 4.0])
    true_bias = 5.0
    y = X @ true_weights + true_bias + noise * torch.randn(num_samples)
    return X, y

# 初始化数据和模型
X, y = create_sample_data()
dataset = TensorDataset(X, y)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# 初始模型配置
input_dim = X.shape[1]
hidden_dims = [64, 32]
output_dim = 1
model = DynamicMLP(input_dim, hidden_dims, output_dim)

# 损失函数和优化器
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.01)

# 训练函数
def train(model, dataloader, criterion, optimizer, epochs=10):
    model.train()
    for epoch in range(epochs):
        epoch_loss = 0.0
        for batch_X, batch_y in dataloader:
            optimizer.zero_grad()
            outputs = model(batch_X)
            loss = criterion(outputs.squeeze(), batch_y)  # Ensure shapes match
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item() * batch_X.size(0)
        print(f"Epoch {epoch+1}/{epochs}, Loss: {epoch_loss/len(X):.4f}")

# 初始训练
print("Initial Training:")
train(model, dataloader, criterion, optimizer, epochs=5)

# 假设模型表现不佳，决定增加一个隐藏层
print("Adding a new hidden layer:")
model.add_layer(16)
train(model, dataloader, criterion, optimizer, epochs=5)

# 如果模型过拟合，可以尝试移除一个隐藏层
print("Removing the last hidden layer:")
model.remove_layer()
train(model, dataloader, criterion, optimizer, epochs=5)