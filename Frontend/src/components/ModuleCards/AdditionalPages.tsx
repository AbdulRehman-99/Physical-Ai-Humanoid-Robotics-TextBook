import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type AdditionalPageItem = {
  title: string;
  description: ReactNode;
  to: string;
  color: string;
};

const AdditionalPageList: AdditionalPageItem[] = [
  {
    title: 'Setup Guide',
    description: (
      <>
        Comprehensive guide to setting up your development environment and getting started with the Physical AI & Humanoid Robotics book.
      </>
    ),
    to: '/docs/setup-guide',
    color: '#8b5cf6', // Purple
  },
  {
    title: 'Glossary',
    description: (
      <>
        Essential terminology and definitions used throughout the Physical AI & Humanoid Robotics documentation.
      </>
    ),
    to: '/docs/glossary',
    color: '#ec4899', // Pink
  },
];

function AdditionalPageCard({title, description, to, color}: AdditionalPageItem) {
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
            Learn More →
          </span>
        </div>
      </Link>
    </div>
  );
}

export default function AdditionalPages(): ReactNode {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className="row">
          {AdditionalPageList.map((props, idx) => (
            <AdditionalPageCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}