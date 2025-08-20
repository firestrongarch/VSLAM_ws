import os
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms
from torchvision import datasets
from torchvision.datasets import MNIST
from torch.utils.data import DataLoader
import torch.utils.data as data
import lightning as L

class Encoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.l1 = nn.Sequential(nn.Linear(28 * 28, 64), nn.ReLU(), nn.Linear(64, 3))

    def forward(self, x):
        return self.l1(x)


class Decoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.l1 = nn.Sequential(nn.Linear(3, 64), nn.ReLU(), nn.Linear(64, 28 * 28))

    def forward(self, x):
        return self.l1(x)

class LitAutoEncoder(L.LightningModule):
    def __init__(self, encoder, decoder):
        super().__init__()
        self.encoder = encoder
        self.decoder = decoder
    def forward(self, x):
        # Define the forward pass
        x = x[0] # Correct the dimension error
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        # tensorboard_logger = self.logger.experiment
        # tensorboard_logger.add_image("generated_images", fake_images, 0)
        return x_hat
    def training_step(self, batch, batch_idx):
        # training_step defines the train loop.
        x, _ = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        loss = F.mse_loss(x_hat, x)
        self.log("train_loss", loss, prog_bar=True, on_step=True, on_epoch=True)
        return loss
    def test_step(self, batch, batch_idx):
        # this is the test loop
        x, _ = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        test_loss = F.mse_loss(x_hat, x)
        self.log("test_loss", test_loss, prog_bar=True, on_step=True, on_epoch=True)
    def validation_step(self, batch, batch_idx):
        # this is the validation loop
        x, _ = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        val_loss = F.mse_loss(x_hat, x)
        self.log("val_loss", val_loss, prog_bar=True, on_step=True, on_epoch=True)
    def predict_step(self, batch, batch_idx, dataloader_idx=0):
        return self(batch)
    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-3)
        return optimizer

# Load data sets
transform = transforms.ToTensor()
train_set = datasets.MNIST(root="datasets/MNIST", download=True, train=True, transform=transform)
test_set = datasets.MNIST(root="datasets/MNIST", download=True, train=False, transform=transform)

if __name__ == "__main__":

    # use 20% of training data for validation
    train_set_size = int(len(train_set) * 0.8)
    valid_set_size = len(train_set) - train_set_size

    # split the train set into two
    seed = torch.Generator().manual_seed(42)
    train_set, valid_set = data.random_split(train_set, [train_set_size, valid_set_size], generator=seed)

    # model
    autoencoder = LitAutoEncoder(Encoder(), Decoder())

    # train model
    trainer = L.Trainer()
    trainer.fit(model=autoencoder, train_dataloaders=DataLoader(train_set), val_dataloaders=DataLoader(valid_set))