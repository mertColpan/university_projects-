#include <stdio.h>
#include <math.h>
#define MAX 20
#define PI 3.14159

void ustel_alma (double M[][MAX] ,int *N);
void  polinom_alma(double M[MAX], int *N);
void logaritma_alma (double M[][MAX] , int *N);
void trigonometrik_ifade_alma (double M[][MAX] , int *N);
void ters_trigonometrik_ifade_alma (double M[][MAX] , int *N);
int fonksiyon(double P[MAX], double U[][MAX], double L[][MAX], double T[][MAX], double TT[][MAX], int Pol, int Ust, int Log, int Tri, int TTri, double x,double *y);
void Bisection(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB);
void regula_falsi(double PR[MAX], double UR[][MAX], double LR[][MAX], double TR[][MAX], double TTR[][MAX], int PolR, int UstR, int LogR, int TriR, int TTriR);
void sayisal_turev (double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB);
void trapez(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB);
void turev_alma (double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB,double x, double *y);
void newton_raphson(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB);
void kare_matris_doldur(double M[][MAX], int N);
void kare_matris_yazdir(double M[][MAX], int N);
void kare_birim_matris_olusturma(double M[][MAX], int N);
void matrisin_tersini_bulma(double M[][MAX], double M2[][MAX], int N);
void deger_matrisi_alma(double M[MAX], int N);
void gauss_eleminasyon (double M[][MAX],double M2[MAX], double M3[MAX] ,int N);
void fonksiyon_deger_alma (double M[][MAX], int N);
int faktoryel(int N);
void gregory_newton (double M[][MAX], int N);
void simpson(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB);
void gauss_seidel(double M[][MAX], double M2[MAX], int N);
//void fonksiyonn(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB);


int main(){
	int secim = 1;
	double sonuc;
	double x;
	double polinom_katsayilari[MAX];
	double ustel_fonsiyonlar[MAX][MAX];
	double logaritmik_fonksiyonlar[MAX][MAX];
	double trigonometrik_fonksiyonlar[MAX][MAX];
	double ters_trigonometrik_fonksiyonlar[MAX][MAX];
	double giris_matrisi[MAX][MAX];
	double cikis_matrisi[MAX][MAX];
	double deger_matrisi[MAX];
	double cevap[MAX];
	double enterpolasyon[MAX][MAX] = {0};
	int N;
	int polinom_derecesi;
	int ustel_sayisi;
	int logaritma_sayisi;
	int trigonometrik_ifade_sayisi;
	int ters_trigonometrik_ifade_sayisi;
	
	while (secim != 0){
		
		printf("cikis: 0\nBisection: 1\nRegula-Falsi: 2\nNewton Raphson: 3\nMatris Tersi: 4\nGauss eleminasyon: 5\nGauss Seidal: 6\nSayisal Turev: 7\nSimpson: 8\nTrapez: 9\nGregory Newton: 10\n");
		printf("Tercihin: ");
		scanf("%d",&secim);
		system("cls");
		switch (secim){
			case 0:
				printf("isleminiz bitmistir.");
				break;
			
			case 1:
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				Bisection(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				break;
			case 2:	
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				regula_falsi(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				break;
			case 3:
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				newton_raphson(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				
				break;
			case 4:
				printf("Matrisin boyutunu giriniz: ");
				scanf("%d",&N);
				kare_matris_doldur(giris_matrisi,N);
				//system("cls");
				printf("Girilen matris:\n\n");
				kare_matris_yazdir(giris_matrisi,N);
				kare_birim_matris_olusturma(cikis_matrisi,N);
				matrisin_tersini_bulma(giris_matrisi,cikis_matrisi,N);
				
				break;
			case 5:
				printf("Matrisin boyutunu giriniz: ");
				scanf("%d",&N);
				kare_matris_doldur(giris_matrisi,N);
					printf("Girilen matris:\n\n");
				kare_matris_yazdir(giris_matrisi,N);
				deger_matrisi_alma(deger_matrisi,N);
				gauss_eleminasyon(giris_matrisi, deger_matrisi,cevap, N);
				
				break;
			case 6:
				printf("Matrisin boyutunu giriniz: ");
				scanf("%d",&N);
				kare_matris_doldur(giris_matrisi,N);
				kare_matris_yazdir(giris_matrisi,N);
				deger_matrisi_alma(deger_matrisi,N);
				gauss_seidel(giris_matrisi, deger_matrisi, N);
				break;
			case 7:
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				sayisal_turev(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				break;
			case 8:
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				simpson(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				break;
			case 9:
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				trapez(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				break;
			case 10:
				printf("Kac tane fonksiyon degeri gireceksiniz: ");
				scanf("%d",&N);
				fonksiyon_deger_alma(enterpolasyon,N);
				gregory_newton(enterpolasyon,N);
				break;
			//Fonksiyonun degerlerini doðruluðunu kontrol ettiðim(trigonometrik,logaritmik..) kod
			/*case 11:
				polinom_alma(polinom_katsayilari, &polinom_derecesi);
				ustel_alma(ustel_fonsiyonlar ,&ustel_sayisi);
				logaritma_alma(logaritmik_fonksiyonlar, &logaritma_sayisi);
				trigonometrik_ifade_alma(trigonometrik_fonksiyonlar, &trigonometrik_ifade_sayisi);
				ters_trigonometrik_ifade_alma(ters_trigonometrik_fonksiyonlar, &ters_trigonometrik_ifade_sayisi);
				fonksiyonn(polinom_katsayilari,ustel_fonsiyonlar,logaritmik_fonksiyonlar,trigonometrik_fonksiyonlar,ters_trigonometrik_fonksiyonlar,polinom_derecesi,ustel_sayisi,logaritma_sayisi,trigonometrik_ifade_sayisi,ters_trigonometrik_ifade_sayisi);
				break;
			*/
		}
		
	}
	
}

void  polinom_alma(double M[MAX], int *N){
	
	int n,i,j,karar;
	
	printf("Polinom alicak misiniz: \nEvet: 1\nHayir: 0\n ");
	scanf("%d",&karar);
	if (karar == 1){
		printf("Polinomun derecesi kac olacak: ");
		scanf("%d",&n);
		//system("cls");
	
		for(i=n+1;i>0;i--){
			printf("%d dereceli terimin katsayisini giriniz: ",i-1);
			scanf("%lf",&M[i-1]);
			//system("cls");
		}
		printf("Eklemis oldugunuz polinomun terimleri \n\n ");
		for(j=n;j>=0;j--){
			printf(" %lf*x^%d, \n",M[j],j);
		}
		*N = n;
	}
}

void ustel_alma (double M[][MAX] , int *N){
	
	int n,i;
	
	printf("Kac tane ustel fonsksiyon yazacaksiniz: ");
	scanf("%d",&n);
	//system("cls");
	
	if(n!=0){
		printf("fonskiyon_katsayisi X (  taban  ^(  x'in katsayisi  * x^x'in derecesi ) )^ fonsiyonun derecesi\n");
		for(i=0;i<n;i++){
			printf("%d. terimin fonskiyonun katsayisini giriniz: ",i+1);
			scanf("%lf",&M[0][i]);
			printf("%d. terimin tabanini giriniz: ",i+1);
			scanf("%lf",&M[1][i]);
			printf("%d. terimin x'in katsayisini giriniz: ",i+1);
			scanf("%lf",&M[2][i]);
			printf("%d. terimin x'in derecesini giriniz: ",i+1);
			scanf("%lf",&M[3][i]);
			printf("%d. terimin fonskiyonun derecesini giriniz: ",i+1);
			scanf("%lf",&M[4][i]);
			//system("cls");
		
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (%lf^(%lf * x^%lf))^%lf\n",M[0][i],M[1][i],M[2][i],M[3][i],M[4][i]);
		}
	}
	
	*N = n;
	
}
void logaritma_alma (double M[][MAX] , int *N){
	
	int n,i;
	
	printf("Kac tane logaritmik fonsksiyon yazacaksiniz: ");
	scanf("%d",&n);
	
	if(n!=0){
		printf("fonskiyon_katsayisi X (  Log_taban * (  x'in katsayisi *  x^x'in derecesi ) )^ fonsiyonun derecesi\n");
		for(i=0;i<n;i++){
			printf("%d. terimin fonskiyonun katsayisini giriniz: ",i+1);
			scanf("%lf",&M[0][i]);
			printf("%d. terimin logaritma tabani giriniz: ",i+1);
			scanf("%lf",&M[1][i]);
			printf("%d. terimin x'in katsayisini giriniz: ",i+1);
			scanf("%lf",&M[2][i]);
			printf("%d. terimin x'in derecesini giriniz: ",i+1);
			scanf("%lf",&M[3][i]);
			printf("%d. terimin fonskiyonun derecesini giriniz: ",i+1);
			scanf("%lf",&M[4][i]);
			//system("cls");
		
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (Log_%lf(%lf * x^%lf))^%lf\n",M[0][i],M[1][i],M[2][i],M[3][i],M[4][i]);
		}
	}
	
	*N = n;
	
}
void trigonometrik_ifade_alma (double M[][MAX] , int *N){
	
	int n,i;
	
	printf("Kac tane trigonometrik fonsksiyon yazacaksiniz: ");
	scanf("%d",&n);
	
	if(n!=0){
		printf("fonskiyon_katsayisi * (trigonometrik ifade  (   x'in katsayisi X x^x'in derecesi   ) )^ fonsiyonun derecesi\n");
		for(i=0;i<n;i++){
			printf("%d. terimin fonskiyonun katsayisini giriniz: ",i+1);
			scanf("%lf",&M[0][i]);
			printf("%d. terimin trigonometrik ifadesini giriniz:\n sin:0 , cos:1 , tan:2; cot:3 \n",i+1);
			scanf("%lf",&M[1][i]);
			printf("%d. terimin x'in katsayisini giriniz: ",i+1);
			scanf("%lf",&M[2][i]);
			printf("%d. terimin x'in derecesini giriniz: ",i+1);
			scanf("%lf",&M[3][i]);
			printf("%d. terimin fonskiyonun derecesini giriniz: ",i+1);
			scanf("%lf",&M[4][i]);
			//system("cls");
		
			if (M[1][i] == 0){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (sin(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}
			if (M[1][i] == 1){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (cos(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}
		if (M[1][i] == 2){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (tan(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}
			if (M[1][i] == 3){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (cot(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}	
		}
	}
	
	*N = n;
	
}
void ters_trigonometrik_ifade_alma (double M[][MAX] , int *N){
	
	int n,i;
	
	printf("Kac tane ters trigonometrik fonsksiyon yazacaksiniz: ");
	scanf("%d",&n);
	
	if(n!=0){
		printf("fonskiyon_katsayisi * ( ters trigonometrik ifade(  x'in katsayisi X x^x'in derecesi  ))^ fonsiyonun derecesi\n");
		for(i=0;i<n;i++){
			printf("%d. terimin fonskiyonun katsayisini giriniz: ",i+1);
			scanf("%lf",&M[0][i]);
			printf("%d. terimin ters trigonometrik ifadesini giriniz:\n arcsin:0 , arccos:1 , arctan:2; arccot:3 \n",i+1);
			scanf("%lf",&M[1][i]);
			printf("%d. terimin x'in katsayisini giriniz: ",i+1);
			scanf("%lf",&M[2][i]);
			printf("%d. terimin x'in derecesini giriniz: ",i+1);
			scanf("%lf",&M[3][i]);
			printf("%d. terimin fonskiyonun derecesini giriniz: ",i+1);
			scanf("%lf",&M[4][i]);
			//system("cls");
		
			if (M[1][i] == 0){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (arcsin(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}
			if (M[1][i] == 1){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (arccos(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}
			if (M[1][i] == 2){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (arctan(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}
			if (M[1][i] == 3){
			
			printf("Eklemis oldugunuz fonksiyon: ");
			printf("%lf * (arccot(%lf * x^%lf))^%lf\n",M[0][i],M[2][i],M[3][i],M[4][i]);
			}	
		}
	}
	
	*N = n;
	
}
int fonksiyon(double P[MAX], double U[][MAX], double L[][MAX], double T[][MAX], double TT[][MAX], int Pol, int Ust, int Log, int Tri, int TTri, double x,double *y){
	
	int i;
	double sum=0,radian,cot,acot,derece,logaritma;
	
	
	for(i=Pol;i>=0;i--){
		sum += P[i]*pow(x,i);
	}
	
	for(i=0;i<Ust;i++){
		
		sum += U[0][i] * pow(((pow(U[1][i],U[2][i] * pow(x,U[3][0])))),U[4][i]);
		
	}
	
	for (i=0;i<Log;i++){
		
		logaritma = log(L[2][i]* pow(x,L[3][i]))/log(L[1][i]);
		
		sum += L[0][i]* pow(logaritma,L[4][i]);
		
	}
	
	
	for (i=0;i<Tri;i++){
	
		
		if (T[1][i] == 0){
			
			radian = (T[2][i] * pow(x,T[3][i]))* (PI/180);
			
			
			sum += T[0][i] * pow(sin(radian),T[4][0]);
		}
		if (T[1][i] == 1){
			
			radian = (T[2][i] * pow(x,T[3][i])) * (PI/180);
			
			sum += T[0][i] * pow(cos(radian),T[4][0]);
		}
		if (T[1][i] == 2){
			
			radian = (T[2][i] * pow(x,T[3][i])) * (PI/180);
			
			sum += T[0][i] * pow(radian,T[4][0]);
		}
		if (T[1][i] == 3){
			
			radian = (T[2][i] * pow(x,T[3][i])) * (PI/180);
			
			cot = 1/tan(radian);
			
			sum += T[0][i] * pow(cot,T[4][0]);
		}	
	}
	for (i=0;i<TTri;i++){
		
		if (TT[1][i] == 0){
		
			derece = asin(TT[2][i] * pow(x,TT[3][i]))*180/PI;
			
			sum += TT[0][i] * pow((derece),TT[4][0]);
		}
		if (TT[1][i] == 1){
		
			derece = acos(TT[2][i] * pow(x,TT[3][i]))*180/PI;
			
			sum += TT[0][i] * pow((derece),TT[4][0]);
		}
		if (TT[1][i] == 2){

			derece = atan(TT[2][i] * pow(x,TT[3][i]))*180/PI;
			
			sum += TT[0][i] * pow((derece),TT[4][0]);
		}
		if (TT[1][i] == 3){
		
			derece = atan(1/(TT[2][i] * pow(x,TT[3][i])))*180/PI;
			
			sum += TT[0][i] * pow((derece),TT[4][0]);
		}	
	}
	
	
	
	*y = sum;
}

void Bisection(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB){
	
	double  baslangic,bitis,orta,hata_miktari,fbas,fbit,fort,fonksiyon_degeri;
	int iterasyon,karar,i,tekrar=0,kontrol=0,kontrol2 = 0, kontrol3 = 0, kontrol4 = 0;
	
	while (kontrol == 0){
		printf("Baslama degerini giriniz: ");
		scanf("%lf",&baslangic);
		//system("cls");
		printf("bitis degerini giriniz: ");
		scanf("%lf",&bitis);
		//system("cls");
		if(baslangic >= bitis){
			printf("baslangic degeri bitis degerine esit ya da buyuk olamaz\n");
		}
		else{
			kontrol = 1;
		}
	}
		
	while (kontrol2 == 0){
		printf("Hata miktarini giriniz: ");
		scanf("%lf",&hata_miktari);
		//system("cls");
		if (hata_miktari<=0){
			printf("Hata miktari 0 dan kucuk ya da esit olamaz\n\n");
		}
		else{
			kontrol2 = 1;
		}
	}
	while (kontrol3 == 0){
		printf("Durma kosulu:\nf(x) <= hata miktari    : 1\n(bitis - baslangic)/2^n : 2\n");
		scanf("%d",&karar);
		//system("cls");
		if (karar != 1 && karar != 2 ){
			
			printf("sadece 1 ya da 2 degerini girebilirsiniz.\n\n");
			
		}
		else{
			
			kontrol3 = 1;
			
		}
	}
	while (kontrol4 == 0){
		printf("Maksimum iterasyon sayisini giriniz: ");
		scanf("%d",&iterasyon);
		//system("cls");
		if (iterasyon<=0){
			
			printf("iterasyon degeri 0 dan buyuk olmalidir\n\n");
			
		}
		else{
			
			kontrol4 = 1;
			
		}
	}
	
	if (karar == 1){
		
		do {
			
			tekrar += 1;
			orta = (baslangic + bitis) /2;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic,&fonksiyon_degeri);
			fbas = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,bitis,&fonksiyon_degeri);
			fbit = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,orta,&fonksiyon_degeri);
			fort = fonksiyon_degeri;
			printf("baslangic      : %lf\n",baslangic);
			printf("Bitis          : %lf\n",bitis);
			printf("Orta           : %lf\n",orta);
			printf("f(baslangic)   : %lf\n",fbas);
			printf("f(bitis)       : %lf\n",fbit);
			printf("f(orta)        : %lf\n",fort);
			printf("iterasyon      : %d\n\n",tekrar);
			
			if ((fbas*fort) < 0){
				bitis = orta;
			}
			else if((fbit*fort) < 0){
				baslangic = orta;
			}
			if (fort <0){
				
				fort = -fort;
			}

		}while((fort>hata_miktari) && tekrar != iterasyon);
		
		printf("Sonuc: %lf\n\n",orta);
	
	 
	
	}
	if (karar == 2){
		
		do {
			
			tekrar += 1;
			orta = (baslangic + bitis) /2;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic,&fonksiyon_degeri);
			fbas = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,bitis,&fonksiyon_degeri);
			fbit = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,orta,&fonksiyon_degeri);
			fort = fonksiyon_degeri;
			printf("baslangýc      : %lf\n",baslangic);
			printf("Bitis          : %lf\n",bitis);
			printf("Orta           : %lf\n",orta);
			printf("f(baslangic)   : %lf\n",fbas);
			printf("f(bitis)       : %lf\n",fbit);
			printf("f(orta)        : %lf\n",fort);
			printf("iterasyon      : %d\n\n",tekrar);
			
			if ((fbas*fort) < 0){
				bitis = orta;
			}
			else if((fbit*fort) < 0){
				baslangic = orta;
			}
		}while (((bitis-baslangic)/pow(2,tekrar)> hata_miktari) && tekrar != iterasyon);
	
		printf("Sonuc: %lf\n\n",orta);
	 
	
	}
}

void regula_falsi(double PR[MAX], double UR[][MAX], double LR[][MAX], double TR[][MAX], double TTR[][MAX], int PolR, int UstR, int LogR, int TriR, int TTriR){
	
	double baslangic,bitis,hata_miktari,fbas,fbit,fpoint,point,fonksiyon_degeri;
	int iterasyon,i,karar,tekrar=0,kontrol = 0, kontrol2 = 0, kontrol3=0,kontrol4=0;
	
	
	
	
	while (kontrol == 0){
		printf("Baslama degerini giriniz: ");
		scanf("%lf",&baslangic);
		//system("cls");
		printf("bitis degerini giriniz: ");
		scanf("%lf",&bitis);
		//system("cls");
		if(baslangic >= bitis){
			printf("baslangic degeri bitis degerine esit ya da buyuk olamaz\n");
		}
		else{
			kontrol = 1;
		}
	}
		
	while (kontrol2 == 0){
		printf("Hata miktarini giriniz: ");
		scanf("%lf",&hata_miktari);
		//system("cls");
		if (hata_miktari<=0){
			printf("Hata miktari 0 dan kucuk ya da esit olamaz\n\n");
		}
		else{
			kontrol2 = 1;
		}
	}
	while (kontrol3 == 0){
		printf("Durma kosulu:\nf(x) <= hata miktari    : 1\n(bitis - baslangic)/2^n : 2\n");
		scanf("%d",&karar);
		//system("cls");
		if (karar != 1 && karar != 2 ){
			
			printf("sadece 1 ya da 2 degerini girebilirsiniz.\n\n");
			
		}
		else{
			
			kontrol3 = 1;
			
		}
	}
	while (kontrol4 == 0){
		printf("Maksimum iterasyon sayisini giriniz: ");
		scanf("%d",&iterasyon);
		//system("cls");
		if (iterasyon<=0){
			
			printf("iterasyon degeri 0 dan buyuk olmalidir\n\n");
			
		}
		else{
			
			kontrol4 = 1;
			
		}
	}
	
	
	if (karar == 1){
	
		do{
				
			tekrar += 1;
			fonksiyon(PR,UR,LR,TR,TTR,PolR,UstR,LogR,TriR,TTriR,baslangic,&fonksiyon_degeri);
			fbas = fonksiyon_degeri;
			fonksiyon(PR,UR,LR,TR,TTR,PolR,UstR,LogR,TriR,TTriR,bitis,&fonksiyon_degeri);
			fbit = fonksiyon_degeri;
				
			point = ((bitis*fbas)-(baslangic*fbit)) / (fbas-fbit);
				
			fonksiyon(PR,UR,LR,TR,TTR,PolR,UstR,LogR,TriR,TTriR,point,&fonksiyon_degeri);
			fpoint = fonksiyon_degeri;
				
			printf("baslangic      : %lf\n",baslangic);
			printf("Bitis          : %lf\n",bitis);
			printf("Point          : %lf\n",point);
			printf("f(baslangic)   : %lf\n",fbas);
			printf("f(bitis)       : %lf\n",fbit);
			printf("f(point)       : %lf\n",fpoint);
			printf("iterasyon      : %d\n\n",tekrar);
				
			if ((fbas*fpoint) < 0){
				bitis = point;
			}
			else if((fbit*fpoint) < 0){
				baslangic = point;
			}
			if (fpoint <0){
				
				fpoint = -fpoint;
			}
		}while(fpoint>hata_miktari && tekrar != iterasyon);
			printf("Sonuc: %lf\n\n",point);
	}
	if (karar == 2){
	
		do{
				
			tekrar += 1;
			fonksiyon(PR,UR,LR,TR,TTR,PolR,UstR,LogR,TriR,TTriR,baslangic,&fonksiyon_degeri);
			fbas = fonksiyon_degeri;
			fonksiyon(PR,UR,LR,TR,TTR,PolR,UstR,LogR,TriR,TTriR,bitis,&fonksiyon_degeri);
			fbit = fonksiyon_degeri;
				
			point = ((bitis*fbas)-(baslangic*fbit)) / (fbas-fbit);
				
			fonksiyon(PR,UR,LR,TR,TTR,PolR,UstR,LogR,TriR,TTriR,point,&fonksiyon_degeri);
			fpoint = fonksiyon_degeri;
				
			printf("baslangýc      : %lf\n",baslangic);
			printf("Bitis          : %lf\n",bitis);
			printf("Point          : %lf\n",point);
			printf("f(baslangic)   : %lf\n",fbas);
			printf("f(bitis)       : %lf\n",fbit);
			printf("f(point)       : %lf\n",fpoint);
			printf("iterasyon      : %d\n\n",tekrar);
				
			if ((fbas*fpoint) < 0){
				bitis = point;
			}
			else if((fbit*fpoint) < 0){
				baslangic = point;
			}
		}while (((bitis-baslangic)/pow(2,tekrar)> hata_miktari) && tekrar != iterasyon);
		printf("Sonuc: %lf\n\n",point);
	}
	
	
	 
}

void sayisal_turev (double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB){
	
	int i,karar,bos;
	double point,h,f1,f2,fonksiyon_degeri,turev_degeri,kontrol = 0;
	
	while (kontrol == 0){
		printf("Ileri fark: 1\nGeri fark: 2\nMerkezi fark: 3\n");
		scanf("%d",&karar);
		//system("cls");
		
		if((karar != 1) && (karar != 2) && (karar != 3) ){
			
			printf("1,2,3 degerlerini girebilirsiniz\n");
			
		}
		else{
			
			kontrol = 1;
			
		}
	}
	
	switch (karar){
		
		case 1:
			printf("Turevini bilmek istediginiz noktayi giriniz: ");
			scanf("%lf",&point);
			//system("cls");
			printf("Fark araligini giriniz: ");
			scanf("%lf",&h);
			//system("cls");
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,point,&fonksiyon_degeri);
			f1 = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,point+h,&fonksiyon_degeri);
			f2 = fonksiyon_degeri;
			turev_degeri = (f2-f1)/(h);
			
			printf("point         : %lf\n",point);
			printf("fark aralýgý  : %lf\n",h);
			printf("f(point)      : %lf\n",f1);
			printf("f(point+h)    : %lf\n\n",f2);
			
			
			printf("turev degeriniz \n f'(%lf) = %lf \n\n",point,turev_degeri);
			
			break;
		
		case 2:
			printf("Turevini bilmek istediginiz noktayi giriniz: ");
			scanf("%lf",&point);
			//system("cls");
			printf("Fark araligini giriniz: ");
			scanf("%lf",&h);
			//system("cls");
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,point,&fonksiyon_degeri);
			f1 = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,point-h,&fonksiyon_degeri);
			f2 = fonksiyon_degeri;
			turev_degeri = (f1-f2)/(h);
			
			printf("point         : %lf\n",point);
			printf("fark aralýgý  : %lf\n",h);
			printf("f(point)      : %lf\n",f1);
			printf("f(point-h)    : %lf\n\n",f2);
			
			
			printf("turev degeriniz \n f'(%lf) = %lf \n\n",point,turev_degeri);
			
			break;
		
		case 3:
			
			printf("Turevini bilmek istediginiz noktayi giriniz: ");
			scanf("%lf",&point);
			//system("cls");
			printf("Fark araligini giriniz: ");
			scanf("%lf",&h);
			//system("cls");
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,point-h,&fonksiyon_degeri);
			f1 = fonksiyon_degeri;
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,point+h,&fonksiyon_degeri);
			f2 = fonksiyon_degeri;
			turev_degeri = (f1-f2)/(2*h);
			
			printf("point         : %lf\n",point);
			printf("fark aralýgý  : %lf\n",h);
			printf("f(point-h)    : %lf\n",f1);
			printf("f(point+h)    : %lf\n\n",f2);
			
			printf("turev degeriniz \n f'(%lf) = %lf \n\n",point,turev_degeri);
			
			break;
				
		
		
		
	}
	
	
}

void trapez(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB){
	
	double x1,x2,f1,f2,bitis,h,fonksiyon_degeri, alanlar_toplam = 0;
	int n,i,kontrol=0,kontrol2 = 0;
	
	
	while (kontrol == 0){
		printf("Baslama degerini giriniz: ");
		scanf("%lf",&x1);
		//system("cls");
		printf("bitis degerini giriniz: ");
		scanf("%lf",&bitis);
		//system("cls");
		if(x1 >= bitis){
			printf("baslangic degeri bitis degerine esit ya da buyuk olamaz\n");
		}
		else{
			kontrol = 1;
		}
	}
	while (kontrol2 == 0){
		printf("Bolum sayisini giriniz: ");
		scanf("%d",&n);
		//system("cls");
		
		if (n<=0){
			
			printf("bolum sayisi 0 dan buyuk olmalidir\n");
			
		}
		else{
			
			kontrol2 = 1;
			
		}
	}
	h = (bitis - x1)/n;
	x2 = x1 + h;
	
	for(i=0;i<n;i++){
		
		fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x1,&fonksiyon_degeri);
		f1 = fonksiyon_degeri;
		fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x2,&fonksiyon_degeri);
		f2 = fonksiyon_degeri;
		
		
		alanlar_toplam += (h/2)*(f1 + f2);
		
		printf("x1         : %lf\n",x1);
		printf("x2         : %lf\n",x2);
		printf("f(x1)      : %lf\n",f1);
		printf("f(x2)      : %lf\n",f2);
		printf("S(n)       : %lf\n\n",alanlar_toplam);
		x1 = x2;
		x2 = x1 + h;
		
		
	}
	
	printf("Trapez yonetimi sonucu cikan alan:  %lf\n",alanlar_toplam);
	
	
	
	
	
	
	
	
	
	
	
}
void turev_alma (double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB,double x, double *y){
	
	double f1,f2,fonksiyon_degeri,h;
	
	h = 0.0001;
	fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x+h,&fonksiyon_degeri);
	f1 = fonksiyon_degeri;
	fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x-h,&fonksiyon_degeri);
	f2 = fonksiyon_degeri;
	
	*y = (f1-f2)/(2*h);
	
	
	
}

void newton_raphson(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB){
	double fonksiyon_degeri,x1,x2,f1,ft1,turev_degeri,hata_miktari,temp;
	int i,iterasyon, kontrol = 0, kontrol2 = 0,kontrol3 = 0,tekrar = 0;
	
	printf("Baslama degerini giriniz: ");
	scanf("%lf",&x1);
	//system("cls");
		
	while (kontrol3 == 0){
		printf("Hata miktarini giriniz: ");
		scanf("%lf",&hata_miktari);
		//system("cls");
		if (hata_miktari<=0){
			printf("Hata miktari 0 dan kucuk ya da esit olamaz\n\n");
		}
		else{
			kontrol3 = 1;
		}
	}
	while (kontrol2 == 0){
		printf("Maksimum iterasyon sayisini giriniz: ");
		scanf("%d",&iterasyon);
		//system("cls");
		if (iterasyon<=0){
			
			printf("iterasyon degeri 0 dan buyuk olmalidir\n\n");
			
		}
		else{
			
			kontrol2 = 1;
			
		}
	}
	
	do{
		tekrar += 1;
		fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x1,&fonksiyon_degeri);
		f1 = fonksiyon_degeri;
		turev_alma(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x1,&turev_degeri);
		ft1 = turev_degeri;
		x2 = x1 - ((f1)/ft1);
		
		printf("x1         : %lf\n",x1);
		printf("x2         : %lf\n",x2);
		printf("f1         : %lf\n",f1);
		printf("f'(1)      : %lf\n",ft1);
		printf("iterasyon  : %d\n\n",tekrar);

		temp = x2;
		x2 = x1;
		x1 = temp;
		
		
	}while((x1-x2)>hata_miktari && tekrar != iterasyon);
	
	printf("sonuc :%lf\n\n",x1);
	
	
}
void kare_matris_doldur(double M[][MAX], int N){
	
	int i,j;
	
	for(i=0;i<N;i++){
		
		for(j=0;j<N;j++){
			
			printf("[%d] [%d] \n",i,j);
			scanf("%lf",&M[i][j]);
		}
	}
	
	
	
	
}

void kare_matris_yazdir(double M[][MAX], int N){
	
	int i,j;
	
	for(i=0;i<N;i++){
		
		for(j=0;j<N;j++){
			
			printf(" %lf",M[i][j]);
		
		}
		printf("\n");
	}
	printf("\n\n");
	
}

void kare_birim_matris_olusturma(double M[][MAX], int N){
	
	int i,j;
	
	for(i=0;i<N;i++){
		
		for(j=0;j<N;j++){
			
			if (i==j){
				
				M[i][j] = 1;
				
			}
			else{
				
				M[i][j] = 0;
			}
		}
	}
	
	
	
}

void matrisin_tersini_bulma(double M[][MAX], double M2[][MAX], int N){
	
	
	int i,j,t,degis;
	double temp;



	
	for (i=0;i<N;i++){
		
		temp = M[i][i];
		
		for(j=0;j<N;j++){
			
			M[i][j] = M[i][j] / temp;
			M2[i][j] = M2[i][j] / temp;
		}
		for (j=0;j<N;j++){
			
			temp = M[j][i];
			if (i != j){
			
				for (t=0;t<N;t++){
					
					M[j][t] = M[j][t] - (M[i][t] * temp);
					M2[j][t] = M2[j][t] - (M2[i][t] * temp);
				}
			}
		}
	}
	printf("Matrisin tersi: \n\n");
	kare_matris_yazdir(M2,N);
	
	
}

void gauss_eleminasyon (double M[][MAX],double M2[MAX],double M3[MAX] ,int N){
	
	int i,j,t;
	double temp;
	
	printf("Deger matrisi: \n\n");
	
	for (i=0;i<N;i++){
		
		printf("%lf\n",M2[i]);
		
	}
	printf("\n");
	//system("cls");
	
	for (i=0;i<N;i++){
		temp = M[i][i];
		for(j=0;j<N;j++){
			
			M[i][j] = M[i][j] / temp;
			
		}
		
		M2[i] = M2[i] / temp;
		for (j=i+1;j<N;j++){
			
			temp = M[j][i];
			
			
			for (t=0;t<N;t++){
				
				M[j][t] = M[j][t] - (M[i][t] * temp);
					
			}
		
		M2[j] = M2[j] - (M2[i]*temp);
			
			
		}
	}
	
	
	
	printf("Ust ucgen matris durumu: \n\n");
	kare_matris_yazdir(M,N);

	printf("\n");
	
	
	
	printf("\n");
	
	for (i=N-1;i>=0;i--){
		temp = M[i][i];
		for (j=N-1;j>i;j--){
			
			if (i!=N-1){
				M2[i] = M2[i] - (M3[N-2-i] * M[i][j]);
			}
			
			
		}
		M3[N-1-i] = M2[i]/temp;
		
	
	}
	for (i=0;i<N;i++){
		printf("x%d: %lf\n\n",i+1,M3[i]);
	}
	
}

void deger_matrisi_alma(double M[MAX], int N){
	
	int i;
	
	for (i=0;i<N;i++){
		
		printf("Deger matrisinin %d. degerini giriniz: ",i+1);
		scanf("%lf",&M[i]);
		
		
	}
	
	
	
}

void gregory_newton (double M[][MAX], int N){
	
	//system("cls");
	int i,j,faktor;
	double x,y;
	double fark[MAX][MAX];
	double h,k,toplam = 0,carpim=1;
	printf("Fonksiyon degerleri: ");
	for (i=0;i<N;i++){
		
		printf("x%d = %lf\n",i+1,M[0][i]);
		printf("f(x%d) = %lf\n\n",i+1,M[1][i]);
		
		
		
	}
	printf("Hangi noktanin enterpolasyon degerini istersiniz: ");
	scanf("%lf",&x);
	
	//system("cls");
	for (i=0;i<N;i++){
		
		fark[0][i] = M[1][i];
		
	}
	for (i=N-1;i>0;i--){
		
		for(j=0;j<i;j++){
			
			fark[N-i][j] = fark[N-1-i][j+1] - fark[N-1-i][j];
		}
	}
	h = M[0][1] - M[0][0];
	k = (x - M[0][0])/h;
	
	for (i=0;i<N;i++){
		
		for (j=0;j<i;j++){
			
			carpim= carpim * (k-j);
			
		}
		faktor = faktoryel(i);
		y = (fark[i][0] * carpim)/ faktor;
		toplam += y; 
		carpim = 1;
	}
	
	printf("p(x) = %lf\n",toplam);
	
}
void fonksiyon_deger_alma (double M[][MAX], int N){
	
	int i;
	for(i=0;i<N;i++){
		
		printf(" %d. f(x)\n",i+1);
		printf("x: ");
		scanf("%lf",&M[0][i]);
		printf("f(x): ");
		scanf("%lf",&M[1][i]);
		
	}
	
	
	
}

int faktoryel(int N){
	
	int i,carpim=1;
	if (N == 0){
		return 1;
	}
	if (N == 1){
		return 1;
	}
	for (i=2;i<=N;i++){
		
		carpim = carpim * i;
		
	}
	
	return carpim;
	
	
}


void simpson(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB){
	
	double fonksiyon_degeri,baslangic,bitis,orta,bolum,h,f1,f2,f3,f4,alan = 0;
	int karar,i;
	
	printf("Baslangic degerini giriniz: ");
	scanf("%lf",&baslangic);
	printf("Bitis degerini giriniz: ");
	scanf("%lf",&bitis);
	printf("Bolum sayisini giriniz: ");
	scanf("%lf",&bolum);
	printf("Simpson 1/3: 1\nSimpson 3/8: 2\n");
	scanf("%d",&karar);
	//system("cls");
	h = (bitis - baslangic) /bolum;
	if(karar == 1){
		
		while(baslangic < bitis){
		
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic,&fonksiyon_degeri);
			f1= fonksiyon_degeri;

			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic+h,&fonksiyon_degeri);
			f2= fonksiyon_degeri;
		
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic+2*h,&fonksiyon_degeri);
			f3= fonksiyon_degeri;
			
			alan+= (h/3)*(f1+4*f2+f3);
			printf("x1              : %lf\n",baslangic);
			printf("x2              : %lf\n",baslangic+h);
			printf("x3              : %lf\n",baslangic+2*h);
			printf("f(1)            : %lf\n",f1);
			printf("f(2)            : %lf\n",f2);
			printf("f(3)            : %lf\n",f3);
			printf("S(n)            : %lf\n\n",alan);
			
				
			baslangic += 2*h;
		}
		printf("Sonuc = %lf\n\n",alan);
		
	}
	else if (karar == 2){
		
		while(baslangic < bitis){
		
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic,&fonksiyon_degeri);
			f1= fonksiyon_degeri;

			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic+h,&fonksiyon_degeri);
			f2= fonksiyon_degeri;
		
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic+2*h,&fonksiyon_degeri);
			f3= fonksiyon_degeri;
			
			fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,baslangic+3*h,&fonksiyon_degeri);
			f4= fonksiyon_degeri;
			
			alan+= (3*h)*((f1+3*f2+3*f3+f4)/8);
			
			printf("x1              : %lf\n",baslangic);
			printf("x2              : %lf\n",baslangic+h);
			printf("x3              : %lf\n",baslangic+2*h);
			printf("x4              : %lf\n",baslangic+3*h);
			printf("f(1)            : %lf\n",f1);
			printf("f(2)            : %lf\n",f2);
			printf("f(3)            : %lf\n",f3);
			printf("f(4)            : %lf\n",f4);
			printf("S(n)            : %lf\n\n",alan);
			
				
			baslangic += 3*h;
		}
		printf("Sonuc = %lf\n\n",alan);
		
		
		
		
		
	}
	
	
	
	
	
}

void gauss_seidel(double M[][MAX], double M2[MAX], int N){
	
	double x[MAX];
	int i,j=0,k,degis,sira=0,iterasyon ;
	double max=0,temp,toplam =0;
	
	for (i=0;i<N;i++){
		printf("%d. bilinmeyen: ",i+1);
		scanf("%lf",&x[i]);
	}
	printf("Iterasyon sayisini giriniz: ");
	scanf("%d",&iterasyon);
	for (i=0;i<N;i++){
		
		for(j=0;j<N;j++){
			
			if (abs(M[j][i]) > max){
				
				max = abs(M[j][i]);
				degis = j;
			}
		
		
		}
		
		for(j=0;j<N;j++){
			
			temp = M[degis][j];
			M[degis][j] = M[sira][j];
			M[sira][j] = temp;
		}
		temp = M2[degis];
		M2[degis] = M2[sira];
		M2[sira] = temp;
		sira+= 1;
		max =0;
	}
	printf("En buyuk kosegen Matrisi: \n\n");
	kare_matris_yazdir(M,N);
	printf("\nYeni deger matrisi: \n\n");
	kare_matris_yazdir(M,N);
	
	for (i=0;i<iterasyon;i++){
		
		for(j=0;j<N;j++){
			
			for (k=0;k<N;k++){
				
				if(k != j){
					
					toplam += M[j][k] * x[k];
					
				}
				
			}	
			x[j] = (M2[j] - toplam )/M[j][j];
			toplam = 0;
		
		}
		for (j=0;j<N;j++){
			
			printf("x%d = %lf\n",j+1,x[j]);
			
		}
		printf("Iterasyon: %d",i+1);
		printf("\n\n");
	}
	printf("\nSonuc: \n\n");
	for (j=0;j<N;j++){
			
			printf("x%d = %lf\n",j+1,x[j]);
			
		}
	printf("\n");
}

/*void fonksiyonn(double PB[MAX], double UB[][MAX], double LB[][MAX], double TB[][MAX], double TTB[][MAX], int PolB, int UstB, int LogB, int TriB, int TTriB){
	
	double x,fonksiyon_degeri,y;
	
	printf("deger: ");
	scanf("%lf",&x);
	
	fonksiyon(PB,UB,LB,TB,TTB,PolB,UstB,LogB,TriB,TTriB,x,&fonksiyon_degeri);
	
	y = fonksiyon_degeri;
	printf("sonuc: %lf\n\n",y);
	
}
	*/
	
	
	
	
	


