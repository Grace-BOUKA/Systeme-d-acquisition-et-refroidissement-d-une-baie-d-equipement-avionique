# Système d'Acquisition, Supervision & Refroidissement Avionique (STM32L476RG / FreeRTOS)

Contrôleur embarqué temps réel sur **STM32L476RG** assurant l'acquisition périodique des données thermiques, la régulation de ventilation par signal **PWM**, l'émission de télémétrie série vers **Tera Term** via **UART**, et la surveillance matérielle par **Watchdog indépendant (IWDG)**.

## Architecture & Fonctionnalités

- **Acquisition périodique (ADC / capteurs simulés) :** Lecture des mesures thermiques via l'ADC du STM32L476RG (avec support DMA selon configuration) ou valeurs simulées.
- **Régulation thermique (PWM) :** Calcul de l'écart thermique et ajustement progressif du rapport cyclique PWM du ventilateur.
- **Ordonnancement multitâche (FreeRTOS / CMSIS-RTOS v2) :**
  - **Tâche 1 (Priorité haute) :** Acquisition et calcul de régulation PWM.
  - **Tâche 2 (Priorité moyenne) :** Formatage et envoi des trames de télémétrie sur l'UART (accès synchronisé avec fil de message (queue)).
  - **Tâche 3 (Priorité basse) :** Signalisation d'état (LEDs) et diagnostic système.
- **Sécurité & tolérance aux pannes (IWDG) :** Rechargement du Watchdog conditionné au suivi d'exécution des tâches critiques (*task check-in*), avec redémarrage automatique du système en cas de blocage ou non respect d'échéance de tâche.

## Environnement & Matériel

- **Microcontrôleur :** STM32L476RG (carte Nucleo-L476RG)
- **IDE & configuration :** STM32CubeIDE / STM32CubeMX (STM32Cube HAL)
- **RTOS :** FreeRTOS (API CMSIS-RTOS v2)
- **Station sol / terminal série :** Tera Term (sur PC via liaison USB-UART ST-LINK / UART)

## Structure du Dépôt

```text
Projet_systeme_baie_avionique/
├── Core/
│   ├── Inc/                 # En-têtes applicatifs et configurations (main.h, FreeRTOSConfig.h, etc.)
│   ├── Src/                 # Code source applicatif (main.c, freertos.c, interruptions, etc.)
│   └── Startup/             # Code de démarrage (startup_stm32l476rgtx.s)
├── Drivers/                 # CMSIS et pilotes STM32L4xx HAL
├── Middlewares/Third_Party/ # Sources FreeRTOS et couche CMSIS-RTOS v2
├── Projet_systeme_baie_avionique.ioc
├── STM32L476RGTX_FLASH.ld
└── STM32L476RGTX_RAM.ld
