import React from 'react';
import styles from '../pages/index.module.css';

export default function TextbookOverview(): JSX.Element {
  return (
    <section className={styles.overviewSection}>
      <div className="container">
        <h2 className={styles.overviewTitle}>What This Textbook Covers</h2>
        <p className={styles.overviewDescription}>
          This textbook provides an in-depth journey through the essentials of humanoid robotics, 
          bridging the gap between software algorithms and physical robot behavior. 
          You will explore ROS 2-based control, high-fidelity physics simulations in Gazebo and Unity, 
          and advanced perception systems with NVIDIA Isaac Sim. By integrating these tools, 
          this book equips you with the skills to develop truly intelligent, autonomous humanoid agents.
        </p>
      </div>
    </section>
  );
}
