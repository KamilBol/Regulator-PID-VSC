# 🧠 Innowacyjny Bypass Analogowy i Logika Pracy (Bumpless Transfer & Auto-Recovery)

Jednym z największych wyzwań przy modernizacji pracujących linii przemysłowych jest integracja nowych sterowników bez niszczenia oryginalnych, sprawdzonych zabezpieczeń fabrycznych maszyny. Zamiast całkowicie wycinać starą automatykę, zaprojektowałem **transparentny system przełączający (Bypass) z funkcją płynnego przejścia i inteligentnego wznawiania pracy po przeciążeniu.**

## 1. Architektura Sprzętowa (Włącznik PID ON/OFF)
Sercem układu są przekaźniki ze stykami przełącznymi (SPDT - Normalnie Zamknięty NC / Normalnie Otwarty NO). Przejęcie kontroli następuje poprzez kliknięcie na ekranie przycisku **PID (ON/OFF)**.

* **PID OFF (Spoczynek - Styk NC): Praca na Starym Układzie**
  Przekaźnik jest zwolniony. Sygnał sterujący płynie bezpośrednio z oryginalnego regulatora maszyny do falowników. 
  *Działanie starego układu:* Gdy pobór prądu osiągnie próg 38.0A, stary regulator natychmiast zrzuca prędkość podajników z maksymalnych 50 Hz do 17 Hz (do ok. 34% wydajności). Mój nowy układ ESP32 w tym czasie pracuje wyłącznie jako bierny obserwator i przygotowuje się do ewentualnego przejęcia.

* **PID ON (Akcja - Styk NO): Praca na Nowym Regulatorze PID**
  ESP32 podaje napięcie na cewkę przekaźnika. Obwód przełącza się na styk NO. Oryginalny regulator zostaje fizycznie odcięty, a pełną kontrolę przejmuje mój system PID, podając na falowniki własny, płynny sygnał (0-10V) wyliczany przez algorytm.

## 2. Magia Programowa: "Bumpless Transfer" (Bezszarpnięciowe Przejście)
W momencie kliknięcia **PID ON**, stary regulator mógł podawać sygnał np. dla 25 Hz. Gdyby mój DAC wystartował od zera, falownik dostałby nagły rozkaz hamowania, generując potężne uderzenie mechaniczne na sprzęgłach.

**Autorskie rozwiązanie:** Kiedy system jest w stanie `PID OFF`, mikrokontroler poprzez przetwornik ADC (ADS1115) nieustannie podsłuchuje napięcie starego regulatora. Jednocześnie w tle wystawia dokładnie to samo napięcie na swoim wyjściu DAC. Gdy operator włącza układ, styk przeskakuje, ale napięcie sterujące pozostaje identyczne! Przejście jest absolutnie niewidoczne dla mechaniki. Dopiero po chwili algorytm PID zaczyna delikatnie korygować prędkość ślimaków.

## 3. Aktywne Zabezpieczenie i Tryby Wznawiania (AUTO / MAN)
System chroni silnik główny przed zakleszczeniem i przeciążeniem, monitorując prąd przez analizator PZEM-004T. W przypadku przekroczenia twardego limitu (Górne Widełki + Próg Awaryjny), układ natychmiast wyłącza system (wymusza `PID OFF`) i zrzuca styki z powrotem na pozycję NC. Kontrolę odzyskuje stary system, który bezzwłocznie redukuje prędkość maszyny do bezpiecznych 17 Hz.

To, co wydarzy się po zrzucie awaryjnym, zależy wyłącznie od ustawionego przez operatora **Trybu Pracy (AUTO / MAN)**:

* **Tryb MANUAL (MAN):**
  Działa jako sztywne zabezpieczenie. Po zrzuceniu przekaźników system pozostaje w stanie `PID OFF`. Maszyna "kręci" się wolno na starym układzie i oczekuje na fizyczną reakcję operatora. Regulator PID **nie wstanie samoczynnie**, dopóki człowiek nie oceni sytuacji i nie włączy go ponownie ręcznie.

* **Tryb AUTOMATIC (AUTO):**
  Tryb w pełni autonomiczny (Auto-Recovery). Po zrzucie awaryjnym maszyna przechodzi pod kontrolę starego układu, ale ESP32 stale monitoruje prąd. Gdy zator zostanie przepchnięty, a prąd spadnie i osiągnie wartość **Dolnych Widełek (Limitu Wznowienia)**, regulator podejmuje akcję. System samoczynnie wyrównuje napięcia (Bumpless Transfer), ponownie załącza przekaźniki odcinając stare zabezpieczenie i samodzielnie odzyskuje kontrolę nad maszyną.
