import os
from svm_cone_classifier import ConeClassifierSVM

def main():
    # Path to your dataset CSV
    csv_path = '../training_data/cluster_training_data.csv'

    # Path to save the trained SVM model
    model_path = '../svm_weights/svm_cone_classifier.pkl'

    print(f"📂 Training from: {csv_path}")
    print(f"💾 Model will be saved to: {model_path}")

    # Create the classifier
    classifier = ConeClassifierSVM(model_path=model_path)

    # Train and save the model
    classifier.train(csv_path)

if __name__ == '__main__':
    main()
