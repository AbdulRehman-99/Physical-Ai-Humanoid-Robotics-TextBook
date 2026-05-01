import React from 'react';
import styles from '../pages/index.module.css';

export default function AINativeSection(): JSX.Element {
  return (
    <section className={styles.overviewSection} style={{ backgroundColor: 'var(--ifm-background-color)' }}>
      <div className="container">
        <h2 className={styles.overviewTitle}>Why This Textbook is AI-Native & Future-Focused</h2>
        <p className={styles.overviewDescription}>
          In the rapidly evolving landscape of robotics, traditional methods are being augmented and 
          transformed by Physical AI. This textbook is designed with an AI-first mindset, focusing on 
          the integration of large-scale foundation models, generative AI for synthetic data, and 
          end-to-end learning frameworks. We don't just teach you how to build robots; we teach you 
          how to build the brains that will power the next generation of autonomous humanoid agents.
        </p>
      </div>
    </section>
  );
}
