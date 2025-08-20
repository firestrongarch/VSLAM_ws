import train

from lightning.pytorch.loggers import TensorBoardLogger

logger = TensorBoardLogger(save_dir="tensorboard_logs")

data_loader = train.DataLoader(train.test_set)
model = train.LitAutoEncoder.load_from_checkpoint("lightning_logs/version_2/checkpoints/epoch=7-step=384000.ckpt", encoder=train.Encoder(), decoder=train.Decoder())
trainer = train.L.Trainer(logger=logger)
predictions = trainer.predict(model, data_loader)
