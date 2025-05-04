# 🤖 GPT vs DeepSeek Answer Evaluation

This project aims to compare the performance of GPT-4o and DeepSeek in answering user questions, using vector-based similarity analysis and classification models. Embedding models (E5, CosmosE5, and Jina) are used to convert textual data into numerical form for evaluation.

## Getting Started

### Requirements

To run this project, Python and some libraries are required. You can install the necessary libraries with the following command:
```bash
pip install torch sentence-transformers scikit-learn pandas openpyxl transformers
```

### Cloning Repository With Models
```bash
sudo apt install git-lfs
git lfs install
git clone https://github.com/mertColpan/university_projects-.git
cd university_projects-/GPT_DeepSeek_answer_evaluation
git clone https://huggingface.co/intfloat/e5-large-v2
git clone https://huggingface.co/cosmos-ene-cosmos-e5
git clone https://huggingface.co/jinaai/jina-embeddings-v2-base-en
cd jina-embeddings-v2-base-en
git lfs pull
```


## Project Structure & Tasks
### Task 1 – Answer Matching via Embedding Similarity

**Objective:**
Evaluate how accurately GPT-4o and DeepSeek answers match the original (true) answer of a question based on vector similarity.

**Steps:**

    Convert each question and its corresponding answers into embedding vectors using different models (E5, CosmosE5, Jina).

    For each question:

        Compare the original answer vector to all other answers using cosine similarity.

        Rank answers and check whether the original answer appears in:

            Top-1 most similar

            Top-5 most similar

    Metrics:

        Top-1 Accuracy

        Top-5 Accuracy

Each metric is calculated per embedding model.
### Task 2 – Supervised Classification: Which Answer is Better?

**Objective:**
Train a classification model to predict which answer (GPT-4o or DeepSeek) users preferred, based on their vector representations.

**Input Features:**
Various vector combinations are used as features, such as:

    Individual embeddings: s, g, d

    Vector differences: s-g, s-d, g-d

    Norm differences: |s-g|, |s-g| - |s-d|

**Where:**

    s = question embedding

    g = GPT-4o answer embedding

    d = DeepSeek answer embedding

**Label:**
User-assigned labels (1, 2, 3, 4) indicating answer quality.

**Model & Evaluation:**

    Model: Random Forest Classifier

    Metrics: Accuracy, Precision, Recall, F1-Score

### Task 3 – Aggregated Accuracy over Multiple Runs

Objective:
Repeat Task 1 multiple times to obtain more robust accuracy estimates and relate those to user feedback.

**Steps:**

    Run Task 1 5 times for each embedding model.

    Compute average Top-1 and Top-5 accuracies.

    Analyze correlations between model performance and user-provided labels (1–4).

### Task 4 – Feature Importance in Classification

**Objective:**
Identify the most informative vector combinations for classification.

**Steps:**

    Generate all unique combinations (length 3 to 10) of vector features without repetition or permutation duplicates.

    Run classification 50 times for each feature combination.

    For each successful classification:

        Count how often each vector feature appears in the best-performing models.

**Output:**

    Frequency statistics for vector features that consistently contribute to higher classification accuracy.


## Running Tasks Usage

### Task 1
```bash
python3 main.py --model_name E5 --sample_size 1000 --task 1
```

### Task 2
```bash
python3 main.py --model_name cosmosE5 --sample_size 1000 --task 2
```

### Task 3
```bash
python3 main.py --model_name cosmosE5 --sample_size 1000 --task 3
```

### Task 4
```bash
python3 main.py --model_name cosmosE5 --sample_size 1000 --task 4
```