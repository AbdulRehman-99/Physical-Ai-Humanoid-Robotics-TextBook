import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import AdditionalPages from './AdditionalPages';
import styles from './styles.module.css';

type ModuleCardItem = {
  title: string;
  description: ReactNode;
  to: string;
  color: string;
};

const ModuleCardList: ModuleCardItem[] = [
  {
    title: 'Module 1: Fundamentals',
    description: (
      <>
        Introduction to ROS (Robot Operating System), basic concepts, and foundational principles of robotics development.
      </>
    ),
    to: '/docs/module-1-ros/ch1-foundations-humanoid-basics',
    color: '#3b82f6', // Blue
  },
  {
    title: 'Module 2: Perception and Navigation',
    description: (
      <>
        Advanced simulation techniques, sensor integration, and navigation algorithms for autonomous robot movement.
      </>
    ),
    to: '/docs/module-2-simulation/ch3-gazebo-digital-twin-physics',
    color: '#10b981', // Green
  },
  {
    title: 'Module 3: Vision-Language-Action Systems',
    description: (
      <>
        NVIDIA Isaac ecosystem, computer vision, and AI-driven robotic control systems for intelligent automation.
      </>
    ),
    to: '/docs/module-3-nvidia-isaac/ch5-isaac-sim-perception-synthetic-data',
    color: '#8b5cf6', // Purple
  },
  {
    title: 'Module 4: Advanced Topics',
    description: (
      <>
        Cutting-edge integration techniques, advanced AI models, and complex robotic system architectures.
      </>
    ),
    to: '/docs/module-4-advanced-integration/ch7-whisper-llm-vla-planning',
    color: '#f59e0b', // Amber
  },
];

function ModuleCard({title, description, to, color}: ModuleCardItem) {
  return (
    <div className={clsx('col col--3', styles.moduleCardCol)}>
      <Link
        to={to}
        className={clsx(styles.moduleCard, styles.featureCard)}
        style={{borderLeftColor: color}}
      >
        <div className={styles.moduleCardHeader}>
          <Heading as="h3" className={styles.moduleCardTitle}>
            {title}
          </Heading>
        </div>
        <div className={styles.moduleCardContent}>
          <p className={styles.moduleCardDescription}>
            {description}
          </p>
        </div>
        <div className={styles.moduleCardFooter}>
          <span className={styles.moduleCardLink}>
            Explore Module →
          </span>
        </div>
      </Link>
    </div>
  );
}

export default function ModuleCards(): ReactNode {
  return (
    <>
      <section className={styles.modulesSection}>
        <div className="container">
          <Heading as="h2" className={styles.sectionTitle}>
            Explore All Modules
          </Heading>
          <div className="row">
            {ModuleCardList.map((props, idx) => (
              <ModuleCard key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>

      {/* Additional Pages Section */}
      <AdditionalPages />
    </>
  );
}