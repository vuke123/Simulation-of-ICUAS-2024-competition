import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from torchvision import transforms

class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.layer1 = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),  # Input channels = 3 (RGB)
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2))
        self.layer2 = nn.Sequential(
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2))
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(64 * 56 * 56, 128)  # Adjust the input features to match your image size
        self.fc2 = nn.Linear(128, 5)  # Adjust the output features to match your number of classes

    def forward(self, x):
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.flatten(x)
        x = self.fc1(x)
        x = self.fc2(x)
        return x

if __name__ == '__main__':
	
	# Initialize the model
	model = SimpleCNN()

	# Loss and Optimizer
	criterion = nn.CrossEntropyLoss()
	optimizer = optim.Adam(model.parameters(), lr=0.001)

	# Assume X_train, y_train are your training data loaded as PyTorch tensors
	# Convert data to TensorDataset and DataLoader for batch processing
	train_data = TensorDataset(X_train, y_train)
	train_loader = DataLoader(train_data, batch_size=5, shuffle=True)
	
	# Training the model
	num_epochs = 3
	for epoch in range(num_epochs):
	    for images, labels in train_loader:
		outputs = model.forward(images)
		loss = criterion(outputs, labels)

		optimizer.zero_grad()
		loss.backward()
		optimizer.step()

	    print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')
	    
	    
	    
