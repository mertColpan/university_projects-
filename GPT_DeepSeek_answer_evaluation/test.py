from sentence_transformers import SentenceTransformer
from transformers import AutoTokenizer
import torch
import matplotlib.pyplot as plt
from visiual import visualize_top1_top5, visualize_labels, visualize_final_averages
import gc
from transformers import AutoTokenizer, AutoModel
from data_utils import select_n_data
from similarity_evaluation import find_close_answer_with_model_3, find_close_answer_with_model_1_2, calculate_correct_prediction

def test(sample_size, dataframe):
    """
    Bu fonksiyon, verilen bir dataframe ve örnek büyüklüğüyle, farklı modelleri değerlendirir.
    Her model için GPT ve DeepSeek cevaplarının benzerliğini karşılaştırır ve Top-1/Top-5 başarı oranlarını ölçer.
    Sonuçları grafiklerle görselleştirir ve ortalamaları yazdırır.

    Parametreler:
    - sample_size (int): Her model için örnekleme yapılacak veri sayısı.
    - dataframe (pandas.DataFrame): Değerlendirilecek veri seti, sorular, cevaplar ve etiketler içerir.
    """
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    total_counter = 0
    total_values = [0, 0, 0, 0]
    total_gpt_top1 = total_gpt_top5 = 0
    total_deep_top1 = total_deep_top5 = 0
    total_runs = 0

    class_gpt_top1_totals = [0, 0, 0, 0]
    class_gpt_top5_totals = [0, 0, 0, 0]
    class_deep_top1_totals = [0, 0, 0, 0]
    class_deep_top5_totals = [0, 0, 0, 0]

    def run_model_block(model_path, job, model_type, repeat=3):
        """
        Bu fonksiyon, belirli bir model yolu ve türüne göre, modelin doğruluğunu test eder.
        Modelin GPT ve DeepSeek top-1 ve top-5 doğruluklarını hesaplar ve görselleştirir.

        Parametreler:
        - model_path (str): Modelin dosya yolu ya da isim.
        - job (str): Modelin görevi ile ilgili açıklama.
        - model_type (str): Model türü ('sentence_transformer' ya da 'huggingface').
        - repeat (int): Her model için tekrar sayısı. Varsayılan 3.
        """
        nonlocal total_counter, total_values
        nonlocal total_gpt_top1, total_gpt_top5
        nonlocal total_deep_top1, total_deep_top5
        nonlocal total_runs
        nonlocal class_gpt_top1_totals, class_gpt_top5_totals
        nonlocal class_deep_top1_totals, class_deep_top5_totals

        print(f"Running model: {model_path}")
        if model_type == "sentence_transformer":
            model = SentenceTransformer(model_path, device=device)
            model.to(device)
            inference_fn = find_close_answer_with_model_1_2
        else:
            tokenizer = AutoTokenizer.from_pretrained(model_path)
            inference_fn = lambda job, q, a, s, m: find_close_answer_with_model_3(q, a, tokenizer, model_path, device, s)
            model = None

        model_gpt_top1 = model_gpt_top5 = 0
        model_deep_top1 = model_deep_top5 = 0

        # Repeat edilen testlerin her biri için işlem yapılıyor
        for _ in range(repeat):
            questions, gpt_answer, deepseek_answer, label = select_n_data(dataframe, sample_size)

            gpt_score, gpt_top1_list, gpt_top5_list, gpt_top1, gpt_top5 = inference_fn(job, questions, gpt_answer, sample_size, model)
            deep_score, deep_top1_list, deep_top5_list, deep_top1, deep_top5 = inference_fn(job, questions, deepseek_answer, sample_size, model)

            # Doğruluk hesaplamaları
            counter, values, gpt_label_top1, gpt_label_top5, deep_label_top1, deep_label_top5 = calculate_correct_prediction(
                gpt_score, gpt_top1_list, gpt_top5_list,
                deep_score, deep_top1_list, deep_top5_list,
                label, sample_size,
                return_per_label=True  # Bu parametre doğru sonuçları etiket bazında döndürür
            )

            # Toplam değerler güncelleniyor
            total_counter += counter
            total_values = [tv + v for tv, v in zip(total_values, values)]
            total_gpt_top1 += gpt_top1
            total_gpt_top5 += gpt_top5
            total_deep_top1 += deep_top1
            total_deep_top5 += deep_top5
            total_runs += 1

            # Etiket bazında toplama yapılıyor
            class_gpt_top1_totals = [x + y for x, y in zip(class_gpt_top1_totals, gpt_label_top1)]
            class_gpt_top5_totals = [x + y for x, y in zip(class_gpt_top5_totals, gpt_label_top5)]
            class_deep_top1_totals = [x + y for x, y in zip(class_deep_top1_totals, deep_label_top1)]
            class_deep_top5_totals = [x + y for x, y in zip(class_deep_top5_totals, deep_label_top5)]

            model_gpt_top1 += gpt_top1
            model_gpt_top5 += gpt_top5
            model_deep_top1 += deep_top1
            model_deep_top5 += deep_top5

            torch.cuda.empty_cache()
            gc.collect()

        # Her model için top-1 ve top-5 doğruluklarını görselleştiriyoruz
        visualize_top1_top5(
            model_gpt_top1 / repeat,
            model_gpt_top5 / repeat,
            model_deep_top1 / repeat,
            model_deep_top5 / repeat,
            sample_size,
            f'{model_path}'
        )
        del model
        torch.cuda.empty_cache()
        gc.collect()
        plt.show(block=True)

    # Tekrar sayısını belirle
    repeat = 5

    # Modelleri sırayla çalıştırıyoruz
    run_model_block('intfloat/multilingual-e5-large-instruct', 'Given a web search query, retrieve relevant passages that answer the query', "sentence_transformer", repeat)
    run_model_block('ytu-ce-cosmos/turkish-e5-large', 'Given a Turkish search query, retrieve relevant passages written in Turkish that best answer the query', "sentence_transformer", repeat)
    run_model_block('./jina-embeddings-v3', '', "huggingface", repeat)

    # Sonuçları ekrana yazdırıyoruz
    print("\n=== Final Averaged Results Across All Models ===")
    print(f"Total Runs: {total_runs}")
    print(f"Average Counter: {total_counter / total_runs}")
    print(f"Average Values: {[v / total_runs for v in total_values]}")
    print(f"Average GPT Top1: {total_gpt_top1 / total_runs}")
    print(f"Average GPT Top5: {total_gpt_top5 / total_runs}")
    print(f"Average DeepSeek Top1: {total_deep_top1 / total_runs}")
    print(f"Average DeepSeek Top5: {total_deep_top5 / total_runs}")

    print("\n=== Per-Label Average Top-1 and Top-5 Accuracies ===")
    for i in range(4):
        print(f"Label {i+1}:")
        print(f"  GPT Top-1: {class_gpt_top1_totals[i] / total_runs:.4f}")
        print(f"  GPT Top-5: {class_gpt_top5_totals[i] / total_runs:.4f}")
        print(f"  Deep Top-1: {class_deep_top1_totals[i] / total_runs:.4f}")
        print(f"  Deep Top-5: {class_deep_top5_totals[i] / total_runs:.4f}")

    # Görselleştirmeler
    visualize_labels(
        avg_values=[v / total_runs for v in total_values],
    )
    visualize_final_averages(
        gpt_top1=list(a/total_runs for a in class_gpt_top1_totals),
        gpt_top5=list(a/total_runs for a in class_gpt_top5_totals),
        deep_top1=list(a/total_runs for a in class_deep_top1_totals),
        deep_top5=list(a/total_runs for a in class_deep_top5_totals)
    )

    plt.show(block=True)
