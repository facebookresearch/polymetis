import os

import torch
import hydra

from models.mlp import MlpTrainer

@hydra.main(config_path="conf/experiment.yml")
def main(cfg):
    # Load data
    assert cfg.data_dir
    data_path = os.path.join(hydra.utils.get_original_cwd(), cfg.data_dir, cfg.data.filename)
    data = torch.load(data_path)
    print(f"Data loaded from {data_path}")

    # Create & train model
    print("Training model...")
    mlp = MlpTrainer(cfg.model)
    mlp.train(data)

    # Save model
    mlp.save(cfg.model.filename)
    print(f"Model saved to: {os.path.join(os.getcwd(), cfg.model.filename)}")

if __name__ == '__main__':
    main()