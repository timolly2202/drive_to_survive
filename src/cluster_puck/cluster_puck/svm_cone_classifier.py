import pandas as pd
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline
import joblib
import os

class ConeClassifierSVM:
    def __init__(self, model_path='/home/jarred/git/drive_to_survive/src/cluster_puck/svm_weights/svm_cone_classifier_v2.pkl'):
        self.model_path = model_path
        self.model = None
        # UPDATED feature list
        self.feature_names = ['extent_x', 'extent_y', 'aspect_ratio', 'area', 'num_points', 'compactness', 'elongation', 'density']

    def train(self, csv_path):
        """Train the SVM classifier from a CSV dataset and save the model."""
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV file not found: {csv_path}")
        
        df = pd.read_csv(csv_path)
        if 'label' not in df.columns:
            raise ValueError("CSV must contain a 'label' column.")
        
        X = df[self.feature_names]
        y = df['label']

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

        pipeline = make_pipeline(StandardScaler(), SVC(kernel='rbf', probability=True))
        pipeline.fit(X_train, y_train)

        accuracy = pipeline.score(X_test, y_test)
        print(f"✅ SVM trained. Test accuracy: {accuracy:.2f}")

        # Save entire pipeline
        joblib.dump(pipeline, self.model_path)
        print(f"Model saved to {self.model_path}")

    def load(self):
        """Load a previously trained model."""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"No model found at {self.model_path}")
        self.model = joblib.load(self.model_path)
        print(f"📥 Loaded model from {self.model_path}")

    def predict(self, features, threshold=0.95):
        """Predict if the current features correspond to a cone based on probability threshold."""
        if self.model is None:
            raise RuntimeError("Model not loaded or trained.")
        
        features_df = pd.DataFrame([features], columns=self.feature_names)
        prob = self.model.predict_proba(features_df)[0][1]  # Probability it's a cone (label=1)

        return int(prob > threshold)