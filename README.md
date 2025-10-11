# Diseño de un medidor de energía eléctrica inalámbrico operado por batería para el monitoreo en tiempo real de tensión y corriente

El monitoreo en tiempo real de variables eléctricas en entornos residenciales y de laboratorio se ha convertido en un requisito clave para la gestión eficiente de la energía y la evaluación del comportamiento de cargas. Sin embargo, los instrumentos tradicionales suelen presentar limitaciones en portabilidad, conectividad y visualización dinámica, restringiendo su uso a escenarios controlados y a lecturas locales. En respuesta a estas limitaciones, este proyecto propone integrar, en un dispositivo compacto y autónomo, un flujo completo de medición–procesamiento–comunicación que habilita la observabilidad continua de variables eléctricas con requisitos de precisión y consumo ajustados al contexto residencial



La solución integra tres pilares técnicos. Primero, una etapa analógica de acondicionamiento y sensado que reduce, desplaza y filtra la señal de tensión, y que amplifica la señal diferencial proveniente de la resistencia \\emph{shunt} para medición de corriente. Esta etapa prioriza precisión, linealidad y bajo ruido, apoyándose en amplificadores operacionales de precisión y en un convertidor analógico–digital sigma–delta de 24~bits. Segundo, un subsistema de gestión de energía, basado en una batería de ion–litio y un PMIC con funciones de carga, protección y conversión elevadora, que garantiza autonomía y estabilidad del suministro para los picos de consumo asociados a la comunicación inalámbrica. Tercero, una unidad digital centrada en un ESP32 con conectividad Wi-Fi, responsable de la adquisición periódica, el almacenamiento temporal de datos y la transmisión mediante WebSocket a una aplicación web progresiva (PWA) diseñada para consulta y visualización remota.



A lo largo del informe se detallan los criterios de diseño y la selección de componentes (Sección \\ref{sec:desi}), la arquitectura de transmisión de datos (Sección \\ref{sec:trans}) y los resultados de simulación eléctrica obtenidos en LTspice (Sección \\ref{sec:sim}), que permiten validar el desempeño nominal de las etapas fundamentales del sistema.



