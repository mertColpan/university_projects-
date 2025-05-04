from transformers import AutoTokenizer, AutoModel
import torch
import torch.nn.functional as F
from tqdm import tqdm
import embedding_utils
import pandas as pd
import visiual


# Model Jina (transformers tabanlı model) ile soruya en yakın cevabı bulur
def find_close_answer_with_model_3(q, answers, tokenizer, model_path, device, n):
    # Model yükleniyor
    model = AutoModel.from_pretrained(model_path, trust_remote_code=True)
    model.to(device)

    # Sorular ve cevaplar liste haline getiriliyor
    sorular = [i for i in q]
    cevaplar = [str(ans).strip() for ans in answers]

    # Soruların gömülü (embedding) temsilleri çıkarılıyor
    soru_embeddings = [embedding_utils.get_embedding("query: " + soru, model, device, tokenizer) for soru in tqdm(sorular, desc="Sorular gömülüyor")]

    # Cevapların gömülü (embedding) temsilleri çıkarılıyor
    cevap_embeddings = [embedding_utils.get_embedding("query: " + cevap, model, device, tokenizer) for cevap in tqdm(cevaplar, desc="Cevaplar gömülüyor")]

    # Top-1 ve Top-5 başarı oranlarını tutacak değişkenler
    top1, top5 = 0, 0
    top1_list, top5_list, results = [], [], []

    # Her bir soru için en benzer cevapları bulma
    for i, emb_soru in enumerate(tqdm(soru_embeddings, desc="Benzerlikler hesaplanıyor")):
        # Soru ve tüm cevaplar arasındaki kosinüs benzerlikleri hesaplanıyor
        similarities = [F.cosine_similarity(emb_soru, emb_cevap).item() for emb_cevap in cevap_embeddings]
        similarities_tensor = torch.tensor(similarities)

        # En yüksek benzerlik değerine sahip ilk 5 cevap seçiliyor
        topk_indices = torch.topk(similarities_tensor, k=5).indices.tolist()
        results.append(similarities_tensor[topk_indices[0]].item())

        # Top-5 içinde gerçek cevabı arama
        topk_texts = [cevaplar[idx] for idx in topk_indices]
        real_clean = cevaplar[i]

        # Top-1 ve Top-5 başarı kontrolü
        top1_list.append(1 if real_clean == topk_texts[0] else 0)
        top5_list.append(1 if real_clean in topk_texts else 0)
        top1 += top1_list[-1]
        top5 += top5_list[-1]

    # Başarı oranları ekrana yazdırılıyor
    print(f'Top-1 başarı: {(top1/n) * 100:.2f}%')
    print(f'Top-5 başarı: {(top5/n) * 100:.2f}%')

    # Sonuçlar döndürülüyor
    return results, top1_list, top5_list, top1, top5


# Model E5 ve cosmosE5 (sentence-transformer gibi modeller) kullanarak soruya en yakın cevabı bulur
def find_close_answer_with_model_1_2(task, q, answers, n, model):
    # Soru ve cevaplar için input textleri hazırlanıyor
    queries = [embedding_utils.get_detailed_instruct(task, i) for i in q]
    documents = [str(ans).strip() for ans in answers]
    input_texts = queries + documents

    # Sorular ve cevaplar için embedding çıkarılıyor
    embeddings = model.encode(input_texts, convert_to_tensor=True, normalize_embeddings=True, show_progress_bar=True)

    # Sorular ve cevaplar arası benzerlik matrisini hesapla
    scores = (embeddings[:len(queries)] @ embeddings[len(queries):].T) * 100

    # Başarı oranlarını ve skorları tutacak değişkenler
    top1, top5 = 0, 0
    top1_list, top5_list, best_scores = [], [], []

    # Her bir soru için en benzer cevapları bulma
    for index, real_answer in enumerate(tqdm(answers, desc='Benzerlikler hesaplanıyor')):
        topk_indices = torch.topk(scores[index], k=5).indices.tolist()
        best_scores.append(scores[index][topk_indices[0]])

        # Top-5 içinde gerçek cevabı arama
        topk_texts = [documents[i].strip() for i in topk_indices]
        real_clean = str(real_answer).strip()

        # Top-1 ve Top-5 başarı kontrolü
        top1_list.append(1 if real_clean == topk_texts[0] else 0)
        top5_list.append(1 if real_clean in topk_texts else 0)
        top1 += top1_list[-1]
        top5 += top5_list[-1]

    # Başarı oranları ekrana yazdırılıyor
    print(f'Top-1 başarı: {(top1/n) * 100:.2f}%')
    print(f'Top-5 başarı: {(top5/n) * 100:.2f}%')

    # Sonuçlar döndürülüyor
    return best_scores, top1_list, top5_list, top1, top5


# Model sonuçlarına göre doğru tahminleri ve sınıf bazlı istatistikleri hesaplar
def calculate_correct_prediction(
      gpt_scores, gpt_top1_list, gpt_top5_list,
      deep_scores, deep_top1_list, deep_top5_list,
      label, n,
      return_per_label=False):
    
    # Toplam doğru tahmin sayısı ve her sınıf için doğru sayısı
    counter, values = 0, [0, 0, 0, 0]
    class_indices = {1: [], 2: [], 3: [], 4: []}  # Her label için indeks listesi

    # Veriler sınıflara göre ayrılıyor
    for i in range(n):
        class_indices[label[i]].append(i)

        # Karar ağacı: GPT ve Deep sonuçlarına göre doğru tahmin kontrolü
        if gpt_scores[i] > 0.70:
            if gpt_top1_list[i] and gpt_top5_list[i]:
                if deep_top1_list[i] and deep_top5_list[i]:
                    if deep_scores[i] > 0.70:
                        if label[i] == 3: counter += 1; values[2] += 1
                    else:
                        if label[i] == 1: counter += 1; values[0] += 1
                else:
                    if label[i] == 1: counter += 1; values[0] += 1
            else:
                if deep_top1_list[i] and deep_top5_list[i]:
                    if deep_scores[i] > 0.70:
                        if label[i] == 2: counter += 1; values[1] += 1
                    else:
                        if label[i] == 4: counter += 1; values[3] += 1
        else:
            if deep_top1_list[i] and deep_top5_list[i]:
                if deep_scores[i] > 0.70:
                    if label[i] == 2: counter += 1; values[1] += 1
                else:
                    if label[i] == 4: counter += 1; values[3] += 1
            else:
                if label[i] == 4: counter += 1; values[3] += 1

    # İstenirse her sınıf için top-1 ve top-5 başarı oranları hesaplanır
    if return_per_label:
        gpt_top1_avg = []
        gpt_top5_avg = []
        deep_top1_avg = []
        deep_top5_avg = []

        for cls in [1, 2, 3, 4]:
            idxs = class_indices[cls]
            gpt_top1_avg.append(sum(gpt_top1_list[i] for i in idxs) / len(idxs) if idxs else 0)
            gpt_top5_avg.append(sum(gpt_top5_list[i] for i in idxs) / len(idxs) if idxs else 0)
            deep_top1_avg.append(sum(deep_top1_list[i] for i in idxs) / len(idxs) if idxs else 0)
            deep_top5_avg.append(sum(deep_top5_list[i] for i in idxs) / len(idxs) if idxs else 0)

            # Sonuçlar ekrana yazdırılıyor
            print(f"Label {cls}: GPT top1 avg = {gpt_top1_avg[-1]:.2f}, GPT top5 avg = {gpt_top5_avg[-1]:.2f}, "
                  f"DeepSeek top1 avg = {deep_top1_avg[-1]:.2f}, DeepSeek top5 avg = {deep_top5_avg[-1]:.2f}")

        # Sınıf bazlı ortalamalar da döndürülüyor
        return counter, values, gpt_top1_avg, gpt_top5_avg, deep_top1_avg, deep_top5_avg

    # Sadece genel doğru tahmin ve değerler döndürülüyor
    return counter, values, [], [], [], []
