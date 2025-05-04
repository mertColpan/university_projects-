from transformers import AutoTokenizer, AutoModel
import torch
from embedding_utils import create_vectors, create_input_vectors
from data_utils import read_excel, select_n_data
from similarity_evaluation import find_close_answer_with_model_3, find_close_answer_with_model_1_2, calculate_correct_prediction 
from model_traning import train_model_with_random_forest, train_model_with_logistic_regression
from sentence_transformers import SentenceTransformer
from collections import defaultdict, Counter
import test
import argparse
from train_all import train_all

def main():
    # Cihaz ayarı (GPU varsa kullanılır, yoksa CPU)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Komut satırı argümanları tanımlanıyor
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_name", type=str, required=True, help="Model adı (örnek: E5, Jina)")
    parser.add_argument("--sample_size", type=int, default=1000, help="Kaç sample kullanılacak")
    parser.add_argument("--task", type=int, choices=[1, 2, 3, 4], default=1, help="Görev numarası (1 , 2, 3, 4)")
    args = parser.parse_args()

    # Argümanlar değişkenlere atanıyor
    model_name = args.model_name
    sample_size = args.sample_size
    task = args.task

    # Verisetinin okunması
    datapath = './data/data.xlsx'
    dataframe = read_excel(datapath)
    
    # Belirli sayıda soru, cevap ve etiket seçiliyor
    questions, gpt_answer, deepseek_answer, label = select_n_data(dataframe, sample_size)

    # Görev 1: Cevap benzerlik değerlendirmesi yapılıyor
    if(task == 1):
        if(model_name == 'E5'):
            # E5 model yükleniyor
            job = 'Given a web search query, retrieve relevant passages that answer the query'
            model_path = 'intfloat/multilingual-e5-large-instruct'
            model = SentenceTransformer(model_path, device=device)
            model.to(device)

            # GPT ve Deepseek cevapları için skorlar hesaplanıyor
            gpt_score, gpt_top1, gpt_top5, _ , _ = find_close_answer_with_model_1_2(job, questions, gpt_answer, sample_size, model)
            deepseek_score, deepseek_top1, deepseek_top5, _ , _ = find_close_answer_with_model_1_2(job, questions, deepseek_answer, sample_size, model)

        elif(model_name == 'cosmosE5'):
            # Türkçe E5 modeli yükleniyor
            job = 'Given a Turkish search query, retrieve relevant passages written in Turkish that best answer the query'
            model_path = 'ytu-ce-cosmos/turkish-e5-large'
            model = SentenceTransformer(model_path, device=device)
            model.to(device)

            # GPT ve Deepseek cevapları için skorlar hesaplanıyor
            gpt_score, gpt_top1, gpt_top5, _ , _ = find_close_answer_with_model_1_2(job, questions, gpt_answer, sample_size, model)
            deepseek_score, deepseek_top1, deepseek_top5, _ , _ = find_close_answer_with_model_1_2(job, questions, deepseek_answer, sample_size, model)
            
        else:
            # Jina embeddings modeli yükleniyor
            model_path = "./jina-embeddings-v3"
            tokenizer = AutoTokenizer.from_pretrained(model_path)
            
            # GPT ve Deepseek cevapları için skorlar hesaplanıyor
            gpt_score, gpt_top1, gpt_top5, _, _ = find_close_answer_with_model_3(questions, gpt_answer, tokenizer, model_path, device, sample_size)
            deepseek_score, deepseek_top1, deepseek_top5, _ , _ = find_close_answer_with_model_3(questions, deepseek_answer, tokenizer, model_path, device, sample_size)

        # Elde edilen skorlarla doğru tahmin oranı hesaplanıyor
        calculate_correct_prediction(gpt_score, gpt_top1, gpt_top5, deepseek_score, deepseek_top1, deepseek_top5, label, sample_size)
    
    # Görev 2: Cümle vektörlerinden input feature oluşturulup logistic regression ile model eğitiliyor
    elif(task == 2):
        if(model_name == 'E5'):
            # E5 modeli yükleniyor
            model_path = 'intfloat/multilingual-e5-large-instruct'
            model = SentenceTransformer(model_path, device=device)
            model.to(device)
        elif(model_name == 'cosmosE5'):
            # Türkçe E5 modeli yükleniyor
            model_path = 'ytu-ce-cosmos/turkish-e5-large'
            model = SentenceTransformer(model_path, device=device)
            model.to(device)
        else:
            # Jina embeddings modeli yükleniyor
            model_path = "./jina-embeddings-v3"
            tokenizer = AutoTokenizer.from_pretrained(model_path)
            model = AutoModel.from_pretrained(model_path, trust_remote_code=True)
            model.to(device)

        # Soru, GPT cevabı ve Deepseek cevabından vektörler oluşturuluyor
        s, g, d = create_vectors(gpt_answer, deepseek_answer, questions, model)

        # Vektörlerden input feature seti oluşturuluyor
        input_feature = create_input_vectors(s, g, d)

        # Logistic regression modeli ile eğitim yapılıyor
        train_model_with_logistic_regression(input_feature, label)
    
    # Görev 3: Test fonksiyonu çağırılıyor
    if(task == 3):
        test.test(sample_size, dataframe)
    
    # Görev 4: Çoklu denemelerle en iyi vektör kombinasyonlarını bulma
    if(task == 4):
        # Model yükleniyor
        model_path = 'ytu-ce-cosmos/turkish-e5-large'
        model = SentenceTransformer(model_path, device=device)
        model.to(device)

        num_runs = 2    # Kaç tekrar yapılacak
        vector_counter = Counter()  # Vektör kombinasyonlarının sayısını tutacak

        # Belirlenen sayıda tekrar yapılır
        for i in range(num_runs):
            print(f"🔁 {i+1}. tekrar")
            
            # Her seferinde farklı sample seçiliyor
            questions, gpt_answer, deepseek_answer, labels = select_n_data(dataframe, sample_size)
            s_embed, g_embed, d_embed = create_vectors(gpt_answer, deepseek_answer, questions, model)
            
            # Eğitim ve en iyi kombinasyonun bulunması
            best_combo = train_all(s_embed, g_embed, d_embed, labels) 
            if best_combo:
                vector_counter.update(best_combo)  # Sayacı güncelle
        
        # Sonuçlar yazdırılıyor
        print("\n🏁 50 tekrar sonunda en çok kullanılan vektörler:")
        for k, v in vector_counter.most_common():
            print(f"{k}: {v} kez")

# Programın başlangıç noktası
if __name__ == '__main__':
    main()
